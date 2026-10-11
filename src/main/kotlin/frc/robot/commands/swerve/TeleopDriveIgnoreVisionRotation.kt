package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.DashboardNumberPublisher
import frc.robot.utils.angle
import frc.robot.utils.metersPerSecond
import frc.robot.utils.radians
import frc.robot.utils.radiansPerSecond
import frc.robot.utils.rotation2d
import kotlin.math.pow

/**
 * A command that controls the swerve drive using joystick inputs.
 *
 * @param getVForward The x velocity of the robot.
 * @param getVStrafe The y velocity of the robot.
 * @param getOmega The angular velocity of the robot.
 * @param slowMode Double supplier that ranges from 0 (normal top speed) to 1 (very reduced speed)
 * @see Drivetrain
 */
class TeleopDriveIgnoreVisionRotation(
    private val getVForward: () -> Double,
    private val getVStrafe: () -> Double,
    private val getOmega: () -> Double,
    private val slowMode: () -> Double,
) : DriveManager.DriveRequestBase() {
    override var vx: LinearVelocity? = 0.0.metersPerSecond
    override var vy: LinearVelocity? = 0.0.metersPerSecond
    override var omega: AngularVelocity? = 0.0.radiansPerSecond
    override val priority: Int = DriverPriority.BASE_TELEOP.ordinal
    override val allowRobotOrientedFallback: Boolean = true
    private var angleOffset: Angle = 0.0.radians

    /** False when the gyro was unreadable at init; the command then degrades to plain teleop. */
    private var hasGyro = true

    override fun initialize() {
        super.initialize()
        val yaw = Drivetrain.rawYaw
        if (yaw == null) {
            // Gyro unreliable: fall back to pose-relative driving instead of assuming zero.
            hasGyro = false
        } else {
            hasGyro = true
            angleOffset = Drivetrain.pose.rotation.angle - yaw
        }
    }

    private var forwardVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    private var strafeVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    private var angVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    private var slowModeCalc: Double by DashboardNumberPublisher(0.0, "Teleop/")

    override fun execute() {
        forwardVelocity = getVForward()
        strafeVelocity = getVStrafe()
        angVelocity = getOmega()
        slowModeCalc = slowMode()

        forwardVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.8))
        strafeVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.8))
        angVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.85))

        val yaw = Drivetrain.rawYaw
        if (yaw == null || !hasGyro) {
            // No trustworthy gyro reading: behave like plain field-relative teleop.
            vx = Drivetrain.Constants.MAX_SPEED * forwardVelocity
            vy = Drivetrain.Constants.MAX_SPEED * strafeVelocity
        } else {
            // Rotate the unitless stick vector, then scale into velocities (same shape as the
            // fallback above). Note: Translation2d.measureX/Y are Distances, not velocities.
            val rotated =
                Translation2d(forwardVelocity, strafeVelocity)
                    .rotateBy((Drivetrain.pose.rotation.angle - (yaw + angleOffset)).rotation2d)

            vx = Drivetrain.Constants.MAX_SPEED * rotated.x
            vy = Drivetrain.Constants.MAX_SPEED * rotated.y
        }
        omega = Drivetrain.Constants.MAX_ANGULAR_SPEED * angVelocity
    }
}
