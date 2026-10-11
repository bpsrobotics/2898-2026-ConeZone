package frc.robot.subsystems

import edu.wpi.first.networktables.BooleanPublisher
import edu.wpi.first.networktables.BooleanSubscriber
import edu.wpi.first.networktables.IntegerPublisher
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringArrayPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Centralized robot health registry.
 *
 * Every fallible device (motors, absolute/incremental encoders, IMUs, cameras, ...) registers an
 * [Source] here. Once per cycle [Alerts] polls all sources and publishes the active errors and
 * warnings to NetworkTables under the `Alerts` table, replacing the old per-vendor error sets (e.g.
 * `SparkWrapper.sparksWithErrors`).
 *
 * - `Alerts/errors`: string array of `"[source] message"` for conditions that make data unusable.
 * - `Alerts/warnings`: string array for degraded-but-usable conditions (sticky faults, disabled
 *   subsystems, ...).
 * - `Alerts/clearFaults`: set to `true` from a dashboard to call `clearFaults` (or equivalent) on
 *   every registered source. Automatically reset to `false` once handled.
 *
 * Note: these live at the NT root (`/Alerts/...`) via `getTable`, not under `/SmartDashboard`
 * (where the `Dashboard*` tunables and `putData` buttons live). Read them from the same tree.
 */
object Alerts : SubsystemBase() {
    enum class Level {
        ERROR,
        WARNING,
    }

    data class Alert(val source: String, val message: String, val level: Level) {
        override fun toString(): String = "[$source] $message"
    }

    /**
     * A fallible device or subsystem that contributes to [Alerts].
     *
     * - [getAlerts] must never throw for a merely-disconnected device; throw only on monitor bugs
     *   (thrown exceptions are converted to errors automatically). Return an [Alert] with
     *   [Level.ERROR] when readings from this source are unreliable and consumers must fall back,
     *   [Level.WARNING] when degraded but usable.
     * - [clearFaults] must clear sticky faults / retry initialization, and never throw.
     */
    interface Source {
        val alertLabel: String

        fun clearFaults()

        fun getAlerts(): List<Alert>
    }

    private val table = NetworkTableInstance.getDefault().getTable("Alerts")
    private val errorsPublisher: StringArrayPublisher =
        table.getStringArrayTopic("errors").publish()
    private val warningsPublisher: StringArrayPublisher =
        table.getStringArrayTopic("warnings").publish()
    private val errorCountPublisher: IntegerPublisher =
        table.getIntegerTopic("errorCount").publish()
    private val warningCountPublisher: IntegerPublisher =
        table.getIntegerTopic("warningCount").publish()
    private val clearFaultsPublisher: BooleanPublisher =
        table.getBooleanTopic("clearFaults").publish()
    private val clearFaultsSubscriber: BooleanSubscriber =
        table.getBooleanTopic("clearFaults").subscribe(false)

    private val sources = mutableListOf<Source>()

    /** Messages already reported to the DriverStation, so we only complain on rising edges. */
    private var reportedMessages = setOf<String>()

    init {
        clearFaultsPublisher.set(false)
        SmartDashboard.putData("Alerts/ClearFaults", clearAllFaultsCommand())
    }

    /** Registers a health source. Duplicate labels are replaced. */
    @Synchronized
    fun register(source: Source) {
        sources.removeAll { it.alertLabel == source.alertLabel }
        sources += source
    }

    /**
     * Registers a health source from lambdas.
     *
     * @param label short device name, e.g. `"Spark11"` or `"Drivetrain/IMU"`.
     * @param clearFaults clears sticky faults / retries init; exceptions are swallowed.
     * @param check returns active alerts; exceptions are converted to errors.
     */
    fun register(label: String, clearFaults: () -> Unit = {}, check: () -> List<Alert>) {
        register(
            object : Source {
                override val alertLabel: String = label

                override fun clearFaults() {
                    runCatching { clearFaults() }
                }

                override fun getAlerts(): List<Alert> = check()
            }
        )
    }

    /** Calls `clearFaults` (or equivalent) on every registered source. Never throws. */
    fun clearAllFaults() {
        synchronized(this) { sources.toList() }
            .forEach { source ->
                runCatching { source.clearFaults() }
            }
    }

    /** Dashboard-button command that clears faults on everything. */
    fun clearAllFaultsCommand(): Command =
        Commands.runOnce(::clearAllFaults).withName("Alerts/ClearFaults")

    /** Polls every source right now. Never throws; monitor crashes become errors. */
    fun activeAlerts(): List<Alert> =
        synchronized(this) { sources.toList() }
            .flatMap { source ->
                runCatching { source.getAlerts() }
                    .getOrElse {
                        listOf(
                            Alert(
                                source.alertLabel,
                                "health monitor crashed: ${it.message}",
                                Level.ERROR,
                            )
                        )
                    }
            }

    override fun periodic() {
        if (clearFaultsSubscriber.get(false)) {
            clearAllFaults()
            clearFaultsPublisher.set(false)
        }

        val alerts = activeAlerts()
        val errors = alerts.filter { it.level == Level.ERROR }.map(Alert::toString)
        val warnings = alerts.filter { it.level == Level.WARNING }.map(Alert::toString)
        errorsPublisher.set(errors.toTypedArray())
        warningsPublisher.set(warnings.toTypedArray())
        errorCountPublisher.set(errors.size.toLong())
        warningCountPublisher.set(warnings.size.toLong())

        val current = (errors + warnings).toSet()
        val fresh = current - reportedMessages
        for (message in fresh.sorted()) {
            if (message in errors) DriverStation.reportError("Alerts: $message", false)
            else DriverStation.reportWarning("Alerts: $message", false)
        }
        reportedMessages = current
    }
}
