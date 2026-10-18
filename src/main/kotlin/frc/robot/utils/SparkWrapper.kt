package frc.robot.utils

import com.revrobotics.PersistMode
import com.revrobotics.REVLibError
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.Alerts

class SparkWrapper(
    val deviceId: Int,
    val motorType: MotorType,
    configurer: SparkMaxConfig.() -> Unit,
) : Sendable, Alerts.Source {
    override val alertLabel: String = "Spark$deviceId"

    private var motor: SparkMax? = null
    lateinit var initError: REVLibError
    val config = SparkMaxConfig()

    var dashboardControl = false
    var requestPower = 0.0

    var lastControl = 0.0

    /**
     * Shaft position, or `null` when the controller is unreachable. Consumers must fall back or
     * shut off instead of assuming zero.
     */
    @Suppress("MemberVisibilityCanBePrivate", "unused")
    val position: Angle?
        get() = runCatching { motor?.encoder?.position?.rotations }.getOrNull()

    /**
     * Shaft velocity, or `null` when the controller is unreachable. Consumers must fall back or
     * shut off instead of assuming zero.
     */
    @Suppress("MemberVisibilityCanBePrivate", "unused")
    val velocity: AngularVelocity?
        get() = runCatching { motor?.encoder?.velocity?.RPM }.getOrNull()

    /** Output current in amps, or `null` when the controller is unreachable. */
    val outputCurrent: Double?
        get() = runCatching { motor?.outputCurrent }.getOrNull()

    init {
        config.configurer()
        initMotor()
        Alerts.register(this)

        SmartDashboard.putData("Motors/Spark$deviceId", this)
    }

    private fun initMotor() {
        val motor = SparkMax(deviceId, motorType)
        this.motor = motor
        initError =
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        if (initError != REVLibError.kOk) {
            motor.close()
            this.motor = null
        }
    }

    /** Clears sticky faults, retrying initialization when the controller was missing. */
    override fun clearFaults() {
        if (motor == null) initMotor() else runCatching { motor?.clearFaults() }
    }

    override fun getAlerts(): List<Alerts.Alert> {
        val motor = motor
        if (motor == null)
            return listOf(
                Alerts.Alert(
                    alertLabel,
                    "not present (init: ${initError.name})",
                    Alerts.Level.ERROR,
                )
            )
        return runCatching { motor.healthAlerts(alertLabel) }
            .getOrElse {
                listOf(Alerts.Alert(alertLabel, "unreachable: ${it.message}", Alerts.Level.ERROR))
            }
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stopMotor() {
        if (dashboardControl) return
        motor?.stopMotor()
        lastControl = 0.0
    }

    fun set(power: Double) {
        if (dashboardControl) return
        motor?.set(power)
        lastControl = power
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun setVoltage(volts: Double) {
        if (dashboardControl) return
        motor?.setVoltage(volts)
        lastControl = volts
    }

    override fun initSendable(builder: SendableBuilder?) {
        if (builder == null) return
        builder.setSmartDashboardType("SparkWrapper")
        builder.addDoubleProperty("position", { motor?.encoder?.position ?: Double.NaN }, null)
        builder.addDoubleProperty("velocity", { motor?.encoder?.velocity ?: Double.NaN }, null)
        builder.addDoubleProperty("busVoltage", { motor?.busVoltage ?: Double.NaN }, null)
        builder.addDoubleProperty("outputCurrent", { motor?.outputCurrent ?: Double.NaN }, null)
        builder.addDoubleProperty("lastControl", { lastControl }, null)
        builder.addIntegerProperty("id", { deviceId.toLong() }, null)
        builder.addStringProperty("initError", { initError.name }, null)
        builder.addStringProperty(
            "faults",
            { runCatching { motor?.faults?.describe() }.getOrNull() ?: "NotPresent" },
            null,
        )
        builder.addStringProperty(
            "stickyFaults",
            { runCatching { motor?.stickyFaults?.describe() }.getOrNull() ?: "NotPresent" },
            null,
        )
        builder.addStringProperty(
            "warnings",
            { runCatching { motor?.warnings?.describe() }.getOrNull() ?: "NotPresent" },
            null,
        )
        builder.addStringProperty(
            "stickyWarnings",
            { runCatching { motor?.stickyWarnings?.describe() }.getOrNull() ?: "NotPresent" },
            null,
        )
        builder.addBooleanProperty(
            "dashboardControl",
            { dashboardControl },
            { dashboardControl = it },
        )
        builder.addDoubleProperty(
            "requestPower",
            { requestPower },
            {
                requestPower = it
                dashboardControl = true
                motor?.set(requestPower)
            },
        )
        builder.addBooleanProperty("clearFaults", { false }, { if (it) clearFaults() })
    }
}
