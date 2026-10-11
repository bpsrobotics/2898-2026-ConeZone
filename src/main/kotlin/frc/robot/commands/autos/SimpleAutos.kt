package frc.robot.commands.autos

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain

/** Placeholder auto: drives forward at 1 m/s for 2 seconds. */
fun simpleDriveAuto(): Command =
    Drivetrain.driveLive { ChassisSpeeds(1.0, 0.0, 0.0) }.withTimeout(2.0)
