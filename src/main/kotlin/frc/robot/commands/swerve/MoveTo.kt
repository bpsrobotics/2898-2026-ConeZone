package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Pose2d
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.controls.toPID
import frc.robot.utils.metersPerSecond
import frc.robot.utils.radiansPerSecond
import kotlin.math.PI

class MoveTo(val targetPose: () -> Pose2d) : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.MOVE_TO.ordinal
    private val xPID = Drivetrain.Constants.TRANSLATION_PID.toPID()
    private val yPID = Drivetrain.Constants.TRANSLATION_PID.toPID()
    private val rotationPID = Drivetrain.Constants.ROTATION_PID.toPID()

    init {
        rotationPID.enableContinuousInput(-PI, PI)
        xPID.setTolerance(0.1)
        yPID.setTolerance(0.1)
        rotationPID.setTolerance(0.2)
    }

    override fun execute() {
        val target = targetPose()
        xPID.setpoint = target.x
        yPID.setpoint = target.y
        rotationPID.setpoint = target.rotation.radians
        val pose = Drivetrain.pose
        vx = xPID.calculate(pose.x).metersPerSecond
        vy = yPID.calculate(pose.y).metersPerSecond
        omega = rotationPID.calculate(pose.rotation.radians).radiansPerSecond
    }

    override fun isFinished(): Boolean {
        return xPID.atSetpoint() && yPID.atSetpoint() && rotationPID.atSetpoint()
    }
}
