package frc.robot.commands.swerve

import frc.robot.subsystems.Drivetrain
import frc.robot.utils.controls.toPID
import frc.robot.utils.radiansPerSecond
import kotlin.math.PI

class HoldAngle : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.HOLD_ANGLE.ordinal

    private val rotationPID = Drivetrain.Constants.ROTATION_PID.toPID()

    init {
        rotationPID.enableContinuousInput(-PI, PI)
    }

    override fun initialize() {
        super.initialize()
        rotationPID.reset()
        rotationPID.setpoint = Drivetrain.pose.rotation.radians
    }

    override fun execute() {
        omega = rotationPID.calculate(Drivetrain.pose.rotation.radians).radiansPerSecond
    }
}
