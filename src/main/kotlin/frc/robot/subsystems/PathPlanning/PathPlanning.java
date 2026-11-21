package frc.robot.subsystems.PathPlanning;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;

public class PathPlanning extends SubsystemBase {
    public static AutoBuilder auto = new AutoBuilder();

    public PathPlanning() {
        RobotConfig config = null;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                () -> Drivetrain.INSTANCE.getFieldPose(),
                (pose) -> {
                    Drivetrain.INSTANCE.resetOdometry(pose);
                },
                () -> Drivetrain.INSTANCE.getRobotVelocity(),
                (speeds, feedforwards) -> Drivetrain.INSTANCE.drive(speeds, false),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

}