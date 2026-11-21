package frc.robot.subsystems.PathPlanning;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Autos;

public class PathPlanning extends SubsystemBase{
    public 
    RobotConfig config;


    try{
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstraints(5.0, 0.0, 0.0),
            new PIDConstraints(5.0, 0.0, 0.0)
        ),
        config.
        () -> {
            var alliance = DriverStation.getAlliance();
            if(alliance.isPresent())
        }
    )
}
