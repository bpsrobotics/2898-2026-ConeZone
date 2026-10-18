package frc.robot.commands.swerve

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.input.OI
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import frc.robot.utils.DashboardMeasure
import frc.robot.utils.asSeconds
import frc.robot.utils.sec

/**
 * Hold-to-confirm gyro reset, bound to [OI.resetGyro].
 * - If vision odometry is on at press time, the hold is long ([longHold]) and firing also turns the
 *   YAGSL vision feed off, leaving odometry relative to the zeroed heading for the rest of the
 *   match. Full vision stays alive: tag-relative aligners don't need the field frame, so the driver
 *   simply doesn't call them.
 * - If the feed is already off (or never worked), the hold is short ([shortHold]) and firing just
 *   re-zeroes the gyro.
 *
 * Slight buzz while arming, firm buzz once fired so the driver knows to release. Requires [OI] only
 * for rumble — deliberately not [Drivetrain], so holding the button never cancels driving.
 */
class ResetGyroHold : Command() {
    private val timer = Timer()
    private var fired = false

    /** True when the vision feed is on at press time, so firing also disables it. */
    private var disablesVision = true

    private var longHold: Time by DashboardMeasure(Units.Seconds, 2.0.sec, "ResetGyroHold")
    private var shortHold: Time by DashboardMeasure(Units.Seconds, 0.5.sec, "ResetGyroHold")

    init {
        addRequirements(OI)
    }

    override fun initialize() {
        timer.restart()
        fired = false
        disablesVision = Vision.enablePoseEstimation
    }

    val threshold: Time
        get() = if (disablesVision) longHold else shortHold

    override fun execute() {
        if (!fired && timer.hasElapsed(threshold.asSeconds)) {
            fired = true
            if (disablesVision) Vision.enablePoseEstimation = false
            Drivetrain.zeroGyro()
        }
        OI.driverController.setRumble(GenericHID.RumbleType.kBothRumble, if (fired) 0.6 else 0.15)
    }

    override fun isFinished(): Boolean {
        // Give an extra 0.2 seconds for the "finished" vibration
        return timer.hasElapsed(threshold.asSeconds + 0.2)
    }

    override fun end(interrupted: Boolean) {
        OI.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }
}
