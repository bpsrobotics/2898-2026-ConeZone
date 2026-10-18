package frc.robot.subsystems

import edu.wpi.first.hal.PowerDistributionFaults
import edu.wpi.first.hal.PowerDistributionStickyFaults
import edu.wpi.first.networktables.DoubleArrayPublisher
import edu.wpi.first.networktables.DoublePublisher
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Power
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.DashboardBoolean
import frc.robot.utils.amps
import frc.robot.utils.volts
import frc.robot.utils.watts

/**
 * REV PDH power monitor (CAN ID [Constants.PDH_CAN_ID]).
 *
 * Publishes battery voltage, total current, computed total power and per-channel currents to the NT
 * root table `PowerMonitor`, and feeds breaker/brownout/CAN faults into [Alerts] (the global
 * `clearFaults` button clears sticky PDH faults too).
 *
 * Notes:
 * - Total power is computed as voltage × total current: the PDH does not report power or energy
 *   itself (both read 0 on REV hardware), and PDH temperature is likewise unsupported, so none of
 *   those are published.
 * - A live reading near 0 V while the RIO is alive means the PDH is unreachable on CAN, so all
 *   readings report `null` and an error is raised. The voltage gate is skipped in simulation, where
 *   sim power defaults to 0.
 */
object PowerMonitor : SubsystemBase() {
    object Constants {
        const val PDH_CAN_ID = 60

        /** Below this input voltage (outside sim), the PDH is considered unreachable. */
        const val UNREACHABLE_VOLTAGE_VOLTS = 0.5
    }

    private const val DEVICE_LABEL = "PowerMonitor/PDH"

    /** Master off-switch (NetworkTables `PowerMonitor/enabled`). Stops polling when false. */
    var enabled: Boolean by DashboardBoolean(true, "PowerMonitor")

    private var pdh: PowerDistribution? = null

    private val table = NetworkTableInstance.getDefault().getTable("PowerMonitor")
    private val voltagePublisher: DoublePublisher = table.getDoubleTopic("voltage").publish()
    private val totalCurrentPublisher: DoublePublisher =
        table.getDoubleTopic("totalCurrent").publish()
    private val totalPowerPublisher: DoublePublisher = table.getDoubleTopic("totalPower").publish()
    private val channelsPublisher: DoubleArrayPublisher =
        table.getDoubleArrayTopic("channels").publish()

    /** Battery voltage, or `null` when the monitor is off or the PDH is unreachable. */
    val voltage: Voltage?
        get() = livePdh()?.let { pdh -> runCatching { pdh.voltage.volts }.getOrNull() }

    /** Sum of monitored channel currents, or `null` when unavailable. */
    val totalCurrent: Current?
        get() = livePdh()?.let { pdh -> runCatching { pdh.totalCurrent.amps }.getOrNull() }

    /**
     * Computed total power (voltage × total current), or `null` when unavailable. The PDH reports 0
     * for power itself, so this is derived.
     */
    val totalPower: Power?
        get() =
            livePdh()?.let { pdh ->
                runCatching { (pdh.voltage * pdh.totalCurrent).watts }.getOrNull()
            }

    val channelCount: Int
        get() = runCatching { pdh?.numChannels ?: 24 }.getOrDefault(24)

    /** Current of one PDH channel, or `null` when unavailable or out of range. */
    fun channelCurrent(channel: Int): Current? {
        val pdh = livePdh() ?: return null
        if (channel !in 0 until channelCount) return null
        return runCatching { pdh.getCurrent(channel).amps }.getOrNull()
    }

    init {
        initPdh()
        Alerts.register("PowerMonitor", { clearFaults() }) { powerAlerts() }
    }

    override fun periodic() {
        val snap = readSnapshot()?.takeIf { isPlausible(it.voltageVolts) }
        voltagePublisher.set(snap?.voltageVolts ?: Double.NaN)
        totalCurrentPublisher.set(snap?.totalCurrentAmps ?: Double.NaN)
        totalPowerPublisher.set(
            if (snap != null) snap.voltageVolts * snap.totalCurrentAmps else Double.NaN
        )
        channelsPublisher.set(snap?.currentsAmps ?: DoubleArray(0))
    }

    private fun initPdh() {
        pdh =
            runCatching {
                PowerDistribution(Constants.PDH_CAN_ID, PowerDistribution.ModuleType.kRev)
            }
                .getOrNull()
    }

    private fun clearFaults() {
        if (pdh == null) initPdh() else runCatching { pdh?.clearStickyFaults() }
    }

    private data class Snapshot(
        val voltageVolts: Double,
        val totalCurrentAmps: Double,
        val currentsAmps: DoubleArray,
        val faults: PowerDistributionFaults,
        val stickyFaults: PowerDistributionStickyFaults,
    )

    private fun readSnapshot(): Snapshot? {
        if (!enabled) return null
        val pdh = pdh ?: return null
        return runCatching {
            Snapshot(
                pdh.voltage,
                pdh.totalCurrent,
                pdh.allCurrents,
                pdh.faults,
                pdh.stickyFaults,
            )
        }
            .getOrNull()
    }

    /** PDH handle when monitoring and reachable, else `null`. */
    private fun livePdh(): PowerDistribution? {
        if (!enabled) return null
        val pdh = pdh ?: return null
        return runCatching { if (isPlausible(pdh.voltage)) pdh else null }.getOrNull()
    }

    private fun isPlausible(voltageVolts: Double): Boolean =
        RobotBase.isSimulation() || voltageVolts >= Constants.UNREACHABLE_VOLTAGE_VOLTS

    private fun powerAlerts(): List<Alerts.Alert> {
        if (!enabled)
            return listOf(
                Alerts.Alert("PowerMonitor", "disabled by off-switch", Alerts.Level.WARNING)
            )
        if (pdh == null)
            return listOf(Alerts.Alert(DEVICE_LABEL, "not present", Alerts.Level.ERROR))
        val snap =
            readSnapshot()
                ?: return listOf(Alerts.Alert(DEVICE_LABEL, "unreadable", Alerts.Level.ERROR))
        if (!isPlausible(snap.voltageVolts))
            return listOf(
                Alerts.Alert(
                    DEVICE_LABEL,
                    "unreachable on CAN ${Constants.PDH_CAN_ID}",
                    Alerts.Level.ERROR,
                )
            )
        val alerts = mutableListOf<Alerts.Alert>()
        val live = snap.faults
        val sticky = snap.stickyFaults
        if (live.Brownout) alerts += Alerts.Alert(DEVICE_LABEL, "brownout", Alerts.Level.ERROR)
        if (live.HardwareFault)
            alerts += Alerts.Alert(DEVICE_LABEL, "hardware fault", Alerts.Level.ERROR)
        if (live.CanWarning)
            alerts += Alerts.Alert(DEVICE_LABEL, "CAN warning", Alerts.Level.WARNING)
        val liveChannels =
            (0 until snap.currentsAmps.size.coerceAtMost(24)).filter { live.getBreakerFault(it) }
        if (liveChannels.isNotEmpty())
            alerts +=
                Alerts.Alert(
                    DEVICE_LABEL,
                    "breaker fault on channels ${liveChannels.joinToString()}",
                    Alerts.Level.ERROR,
                )
        if (sticky.Brownout && !live.Brownout)
            alerts += Alerts.Alert(DEVICE_LABEL, "brownout recorded", Alerts.Level.STICKY)
        if (sticky.CanWarning && !live.CanWarning)
            alerts += Alerts.Alert(DEVICE_LABEL, "CAN warning recorded", Alerts.Level.STICKY)
        if (sticky.CanBusOff)
            alerts += Alerts.Alert(DEVICE_LABEL, "CAN bus off recorded", Alerts.Level.STICKY)
        if (sticky.HardwareFault && !live.HardwareFault)
            alerts += Alerts.Alert(DEVICE_LABEL, "hardware fault recorded", Alerts.Level.STICKY)
        if (sticky.FirmwareFault)
            alerts += Alerts.Alert(DEVICE_LABEL, "firmware fault recorded", Alerts.Level.STICKY)
        if (sticky.HasReset)
            alerts += Alerts.Alert(DEVICE_LABEL, "device has reset", Alerts.Level.STICKY)
        val stickyChannels =
            (0 until snap.currentsAmps.size.coerceAtMost(24)).filter {
                sticky.getBreakerFault(it) && it !in liveChannels
            }
        if (stickyChannels.isNotEmpty())
            alerts +=
                Alerts.Alert(
                    DEVICE_LABEL,
                    "breaker faults recorded on channels ${stickyChannels.joinToString()}",
                    Alerts.Level.STICKY,
                )
        return alerts
    }
}
