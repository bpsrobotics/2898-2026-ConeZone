package frc.robot.utils

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkMax
import com.studica.frc.AHRS
import frc.robot.subsystems.Alerts
import frc.robot.subsystems.Alerts.Alert
import frc.robot.subsystems.Alerts.Level

/**
 * Vendor-specific health checks that feed [Alerts].
 *
 * Conventions:
 * - An [Level.ERROR] means readings from the device are unreliable; consumers must treat the
 *   corresponding measurement as `null` and fall back or shut off.
 * - A [Level.WARNING] means an active degraded-but-usable condition (calibration, ...).
 * - A [Level.STICKY] means a non-active recorded condition (sticky faults, resets, ...).
 */
fun SparkBase.Faults.describe(): String {
    val result = mutableListOf<String>()
    if (motorType) result.add("MotorType")
    if (can) result.add("CAN")
    if (sensor) result.add("Sensor")
    if (firmware) result.add("Firmware")
    if (escEeprom) result.add("Esc EEPROM")
    if (gateDriver) result.add("GateDriver")
    if (temperature) result.add("Temperature")
    if (other) result.add("Other")
    return result.joinToString()
}

fun SparkBase.Warnings.describe(): String {
    val result = mutableListOf<String>()
    if (escEeprom) result.add("Esc EEPROM")
    if (sensor) result.add("Sensor")
    if (stall) result.add("Stall")
    if (brownout) result.add("Brownout")
    if (overcurrent) result.add("Overcurrent")
    if (extEeprom) result.add("Ext EEPROM")
    if (hasReset) result.add("Reset")
    if (other) result.add("Other")
    return result.joinToString()
}

/** REV fault/warning bitfields only define the low 8 bits. */
private const val SPARK_KNOWN_BITS = 0xFF

/**
 * True when a REV bitfield is all-ones. An absent or mis-ID'd controller reads back every bit set,
 * which describes() would otherwise render as literally every fault at once.
 */
private fun isAbsentReading(rawBits: Int): Boolean =
    (rawBits and SPARK_KNOWN_BITS) == SPARK_KNOWN_BITS

/** Active [Alerts] for a present [SparkMax]; empty when fully healthy. */
fun SparkMax.healthAlerts(label: String): List<Alert> {
    val faults = this.faults
    val warnings = this.warnings
    if (
        faults != null &&
            warnings != null &&
            isAbsentReading(faults.rawBits) &&
            isAbsentReading(warnings.rawBits)
    )
        return listOf(Alert(label, "not connected", Level.ERROR))
    val errorStrings = mutableListOf<String>()
    val warningStrings = mutableListOf<String>()
    val stickyStrings = mutableListOf<String>()
    faults?.let { if (it.describe().isNotEmpty()) errorStrings.add("faults: ${it.describe()}") }
    warnings?.let {
        if (it.describe().isNotEmpty()) warningStrings.add("warnings: ${it.describe()}")
    }
    stickyFaults?.let {
        if (it.describe().isNotEmpty()) stickyStrings.add("sticky faults: ${it.describe()}")
    }
    stickyWarnings?.let {
        if (it.describe().isNotEmpty()) stickyStrings.add("sticky warnings: ${it.describe()}")
    }
    return errorStrings.map { Alert(label, it, Level.ERROR) } +
        warningStrings.map { Alert(label, it, Level.WARNING) } +
        stickyStrings.map { Alert(label, it, Level.STICKY) }
}

/**
 * Registers a REV [SparkMax] (or absent one) with [Alerts].
 *
 * @param getMotor returns the live motor, or `null` when init failed / hardware missing.
 */
fun Alerts.registerSpark(label: String, getMotor: () -> SparkMax?) {
    register(label, { clearUnwrappedDevice(getMotor()) }) {
        unwrappedDeviceAlerts(label, runCatching { getMotor() }.getOrNull())
    }
}

/** Combines connection, fault-bitfield, sticky-bitfield and reset status into [Alerts]. */
fun ctreAlerts(
    label: String,
    connected: Boolean,
    faultBits: Int?,
    stickyBits: Int?,
    hasReset: Boolean,
): List<Alert> {
    if (!connected) return listOf(Alert(label, "not connected", Level.ERROR))
    val alerts = mutableListOf<Alert>()
    if (faultBits != null && faultBits != 0)
        alerts.add(Alert(label, "faults active (field=0x${faultBits.toString(16)})", Level.ERROR))
    if (stickyBits != null && stickyBits != 0)
        alerts.add(
            Alert(
                label,
                "sticky faults recorded (field=0x${stickyBits.toString(16)})",
                Level.STICKY,
            )
        )
    if (hasReset) alerts.add(Alert(label, "device has reset", Level.STICKY))
    return alerts
}

/**
 * Registers any CTRE [ParentDevice] with [Alerts] using only connection + reset status. Prefer the
 * type-specific overloads ([registerTalonFX], [registerCANcoder], [registerPigeon2]) which also
 * report fault bitfields.
 */
fun Alerts.registerCtreDevice(label: String, device: ParentDevice, clearStickyFaults: () -> Unit) {
    register(label, clearStickyFaults) {
        runCatching { ctreAlerts(label, device.isConnected, null, null, device.hasResetOccurred()) }
            .getOrElse { listOf(Alert(label, "unreachable: ${it.message}", Level.ERROR)) }
    }
}

/** Registers a CTRE [TalonFX] with [Alerts], polling its fault and sticky-fault fields. */
fun Alerts.registerTalonFX(label: String, device: TalonFX) {
    register(label, { device.clearStickyFaults() }) {
        ctrePollAlerts(label, device, { device.faultField }, { device.stickyFaultField })
    }
}

/** Registers a CTRE [CANcoder] with [Alerts], polling its fault and sticky-fault fields. */
fun Alerts.registerCANcoder(label: String, device: CANcoder) {
    register(label, { device.clearStickyFaults() }) {
        ctrePollAlerts(label, device, { device.faultField }, { device.stickyFaultField })
    }
}

/** Registers a CTRE [Pigeon2] with [Alerts], polling its fault and sticky-fault fields. */
fun Alerts.registerPigeon2(label: String, device: Pigeon2) {
    register(label, { device.clearStickyFaults() }) {
        ctrePollAlerts(label, device, { device.faultField }, { device.stickyFaultField })
    }
}

/**
 * Polls a CTRE device's fault and sticky-fault fields (one CAN read each). Reports "unreachable"
 * when the signals themselves fail.
 */
fun ctrePollAlerts(
    label: String,
    device: ParentDevice,
    faultSignal: () -> StatusSignal<Int>,
    stickySignal: () -> StatusSignal<Int>,
): List<Alert> {
    val faults = runCatching { faultSignal().refresh() }.getOrNull()
    val sticky = runCatching { stickySignal().refresh() }.getOrNull()
    if (faults == null || sticky == null) return listOf(Alert(label, "unreachable", Level.ERROR))
    if (!faults.status.isOK || !sticky.status.isOK)
        return listOf(Alert(label, "unreachable (${faults.status.name})", Level.ERROR))
    return runCatching {
        ctreAlerts(
            label,
            device.isConnected,
            faults.value,
            sticky.value,
            device.hasResetOccurred(),
        )
    }
        .getOrElse { listOf(Alert(label, "unreachable: ${it.message}", Level.ERROR)) }
}

/**
 * Health alerts for an already-unwrapped vendor device object (e.g. YAGSL `SwerveMotor.getMotor()`
 * / `getAbsoluteEncoder()` / `getIMU()`). Unknown types yield no alerts; pair with a live-read
 * probe for those.
 */
fun unwrappedDeviceAlerts(label: String, raw: Any?): List<Alert> =
    when (raw) {
        null -> listOf(Alert(label, "not present", Level.ERROR))
        is SparkMax ->
            runCatching { raw.healthAlerts(label) }
                .getOrElse { listOf(Alert(label, "unreachable: ${it.message}", Level.ERROR)) }
        is TalonFX -> ctrePollAlerts(label, raw, { raw.faultField }, { raw.stickyFaultField })
        is CANcoder -> ctrePollAlerts(label, raw, { raw.faultField }, { raw.stickyFaultField })
        is Pigeon2 -> ctrePollAlerts(label, raw, { raw.faultField }, { raw.stickyFaultField })
        is AHRS -> runCatching {
                when {
                    !raw.isConnected -> listOf(Alert(label, "not connected", Level.ERROR))
                    raw.isCalibrating ->
                        listOf(Alert(label, "calibrating, heading unreliable", Level.WARNING))
                    else -> emptyList()
                }
            }
                .getOrElse { listOf(Alert(label, "unreachable: ${it.message}", Level.ERROR)) }
        is ParentDevice ->
            runCatching { ctreAlerts(label, raw.isConnected, null, null, raw.hasResetOccurred()) }
                .getOrElse { listOf(Alert(label, "unreachable: ${it.message}", Level.ERROR)) }
        else -> emptyList()
    }

/** Best-effort `clearFaults` (or equivalent) on an unwrapped vendor device object. Never throws. */
fun clearUnwrappedDevice(raw: Any?) {
    runCatching {
        when (raw) {
            is SparkMax -> raw.clearFaults()
            is TalonFX -> raw.clearStickyFaults()
            is CANcoder -> raw.clearStickyFaults()
            is Pigeon2 -> raw.clearStickyFaults()
            else -> {}
        }
    }
}

/** Registers a NavX [AHRS] with [Alerts]. */
fun Alerts.registerNavX(label: String, ahrs: AHRS) {
    register(label, {}) {
        runCatching {
            when {
                !ahrs.isConnected -> listOf(Alert(label, "not connected", Level.ERROR))
                ahrs.isCalibrating ->
                    listOf(Alert(label, "calibrating, heading unreliable", Level.WARNING))
                else -> emptyList()
            }
        }
            .getOrElse { listOf(Alert(label, "unreachable: ${it.message}", Level.ERROR)) }
    }
}

/**
 * Registers a generic sensor with [Alerts] that is healthy iff [isConnected] holds.
 *
 * Use for encoders/IMUs without a richer monitor: while disconnected, consumers must treat the
 * measurement as `null`.
 */
fun Alerts.registerConnectedSensor(
    label: String,
    isConnected: () -> Boolean,
    clearFaults: () -> Unit = {},
) {
    register(label, clearFaults) {
        runCatching { isConnected() }
            .fold(
                onSuccess = {
                    if (it) emptyList() else listOf(Alert(label, "not connected", Level.ERROR))
                },
                onFailure = { listOf(Alert(label, "unreachable: ${it.message}", Level.ERROR)) },
            )
    }
}
