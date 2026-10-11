package frc.robot.commands.swerve

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.DashboardNumberPublisher
import frc.robot.utils.metersPerSecond
import frc.robot.utils.radiansPerSecond
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
class TeleopDrive(
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

        vx = Drivetrain.Constants.MAX_SPEED * forwardVelocity
        vy = Drivetrain.Constants.MAX_SPEED * strafeVelocity
        omega = Drivetrain.Constants.MAX_ANGULAR_SPEED * angVelocity
    }
}
