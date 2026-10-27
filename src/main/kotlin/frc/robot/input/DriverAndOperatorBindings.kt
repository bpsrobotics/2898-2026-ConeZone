package frc.robot.input

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.swerve.CardinalAlign
import frc.robot.commands.swerve.ResetGyroHold
import frc.robot.commands.swerve.TeleopDrive
import frc.robot.commands.vision.alignCone
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import frc.robot.utils.degrees
import frc.robot.utils.getAlliance
import frc.robot.utils.opposite
import frc.robot.utils.sec

@Suppress("Unused")
fun OI.driverAndOperatorBindings() {
    // region Drivetrain
    // Hold to zero the gyro, with a firm rumble on fire. Usage while vision is enabled turns off
    // vision pose estimation and has a heavy rumble to signify this.
    resetGyro.whileTrue(ResetGyroHold())

    // Default drive
    Drivetrain.defaultCommand =
        driveManager.alongWith(
            driveManager.defineDriver(
                TeleopDrive({ -translationY }, { -translationX }, { -turn }, { rightTrigger })
            )
        )

    // Lock pose: put the wheels in an X shape to make it harder for other robots to move ours
    driverController.x().debounce(0.2).whileTrue(Drivetrain.run { Drivetrain.lock() })

    /**
     * B and Y: Cone alignment
     * - B alone: my alliance
     * - Y alone: Bunnies
     * - both: opposite alliance
     */
    driverController
        .b()
        .or(driverController.y())
        .whileTrue(
            alignCone({
                if (driverController.y().asBoolean) {
                    if (driverController.b().asBoolean)
                        Vision.ConePipeline.forAlliance(getAlliance())
                    else Vision.ConePipeline.WHITE
                } else Vision.ConePipeline.forAlliance(getAlliance().opposite())
            })
        )

    // Gyro untrustworthy in teleop: DriveManager drops teleop to robot-oriented, so shake the
    // controller left-right-left-right until field-relative driving is back.
    Trigger { DriverStation.isTeleop() && Drivetrain.enabled && !Drivetrain.fieldOrientedAvailable }
        .onTrue(alternatingRumble().withTimeout(2.0.sec))

    // Cardinal align: Turn to one of the 8 cardinal directions relative to the field
    Trigger { driverController.hid.pov != -1 }
        .whileTrue(driveManager.defineDriver(CardinalAlign { -driverController.hid.pov.degrees }))

    // endregion Drivetrain
}
