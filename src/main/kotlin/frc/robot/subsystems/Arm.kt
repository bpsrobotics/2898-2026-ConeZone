package frc.robot.subsystems

import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.utils.InvertibleDigitalInput
import frc.robot.utils.SparkWrapper
import frc.robot.utils.volts

object Arm : SubsystemBase() {
    object Constants {
        const val MOTOR_ID = 15
        const val PRESENCE_SENSOR_ID = 1
    }

    private val motor =
        SparkWrapper(Constants.MOTOR_ID, MotorType.kBrushless) {
            smartCurrentLimit(30)
            idleMode(IdleMode.kBrake)
            inverted(true)
        }

    private val presenceSensor =
        InvertibleDigitalInput(
            Constants.PRESENCE_SENSOR_ID,
            inverted = true,
        )

    private val sysId = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { voltage -> motor.setVoltage(voltage.`in`(edu.wpi.first.units.Units.Volts)) },
            { log ->
                val entry = log.motor("arm")
                entry.voltage(motor.lastControl.volts)
                motor.position?.let(entry::angularPosition)
                motor.velocity?.let(entry::angularVelocity)
            },
            this,
        ),
    )

    init {
        defaultCommand = stop()
    }

    override fun periodic() {
        SmartDashboard.putBoolean("Arm/presence", presenceSensor.get())
    }

    fun stop(): Command = runOnce { motor.stopMotor() }

    fun runIntake(power: Double): Command = runEnd({ motor.set(power) }, { motor.stopMotor() })

    fun runIntake(power: () -> Double): Command =
        runEnd({ motor.set(power()) }, { motor.stopMotor() })

    fun runIntakeUntilPresent(power: Double): Command =
        runIntake(power).until { presenceSensor.get() }

    fun runIntakeUntilGone(power: Double): Command =
        runIntake(power).until { !presenceSensor.get() }

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command =
        sysId.quasistatic(direction).finallyDo(Runnable { motor.stopMotor() })

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command =
        sysId.dynamic(direction).finallyDo(Runnable { motor.stopMotor() })
}
