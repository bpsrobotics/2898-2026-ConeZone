package frc.robot.input

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.swerve.CardinalAlign
import frc.robot.commands.swerve.LockDrive
import frc.robot.commands.swerve.ResetGyroHold
import frc.robot.commands.swerve.TeleopDrive
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.degrees

@Suppress("Unused")
fun OI.driverAndOperatorBindings() {
    /** Drivetrain */
    // Hold to zero the gyro (long hold also cuts vision odometry while it's on): slight buzz
    // while arming, firm buzz on fire.
    resetGyro.whileTrue(ResetGyroHold())
    // Default drive
    Drivetrain.defaultCommand =
        driveManager.alongWith(
            driveManager.defineDriver(
                TeleopDrive({ -translationY }, { -translationX }, { -turn }, { rightTrigger })
            )
        )

    driverController.x().debounce(0.2).whileTrue(driveManager.defineDriver(LockDrive()))

    // Gyro untrustworthy in teleop: DriveManager drops teleop to robot-oriented, so shake the
    // controller left-right-left-right until field-relative driving is back.
    Trigger { DriverStation.isTeleop() && Drivetrain.enabled && !Drivetrain.fieldOrientedAvailable }
        .whileTrue(gyroFallbackRumble())

    Trigger { driverController.hid.pov != -1 }
        .whileTrue(driveManager.defineDriver(CardinalAlign { -driverController.hid.pov.degrees }))
}
