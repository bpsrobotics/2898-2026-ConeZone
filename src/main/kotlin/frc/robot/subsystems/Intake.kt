package frc.robot.subsystems

import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.SparkWrapper

object Intake : SubsystemBase() {
    object Constants {
        const val MOTOR_ID = 15
    }

    private val motor =
        SparkWrapper(Constants.MOTOR_ID, MotorType.kBrushless) {
            smartCurrentLimit(30)
            idleMode(IdleMode.kBrake)
            inverted(true)
        }

    init {
        defaultCommand = stop()
    }

    fun stop() = runOnce { motor.stopMotor() }

    fun runIntake(power: Double) = runEnd({ motor.set(power) }, { motor.stopMotor() })

    fun runIntake(power: () -> Double) = runEnd({ motor.set(power()) }, { motor.stopMotor() })
}
