package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.*;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static edu.wpi.first.units.Units.Centimeters;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Elevator extends SubsystemBase {

    // Elevator motor configuration
    private final SparkMax leadMotor = new SparkMax(ELEVATOR_LEADER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax followerMotorA = new SparkMax(FOLLOWER_MOTOR_IDS[0], MotorType.kBrushless);
    private final SparkMax followerMotorB = new SparkMax(FOLLOWER_MOTOR_IDS[1], MotorType.kBrushless);
    private final SparkMax followerMotorC = new SparkMax(FOLLOWER_MOTOR_IDS[2], MotorType.kBrushless);
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();
    private final SparkMaxConfig resetConfig = new SparkMaxConfig();
    private final Encoder encoder = new Encoder(ENCODER_LA_ID, ENCODER_LB_ID);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV);
    private Distance targetHeight = Centimeters.of(0.0);
    private boolean isHomed = false;
    private DigitalInput bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_ID);

    private final ProfiledPIDController pid = new ProfiledPIDController(
            kP,
            kI,
            kD,
            ELEVATOR_MOTION_CONSTRAINTS);

    /** Invoke the rites of awakening for ascender. */
    public Elevator() {
        encoder.setDistancePerPulse(ElevatorConstants.ENCODER_PPC);
        followerConfig.follow(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);
        followerMotorA.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotorB.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotorC.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12);

        pid.setTolerance(0.5);

        configureMotors();
    }

    /** Sets motor controller settings in the event of rewrite. */
    private void configureMotors() {
        leadMotor.configure(resetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotorA.configure(resetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Stops elevator by setting lead motor speed to 0.0 */
    public void stop() {
        leadMotor.set(0.0);
    }

    /**
     * Clamps an input Distance's magnitude between MIN_HEIGHT_CM and MAX_HEIGHT_CM.
     */
    public Distance clampToValidHeight(Distance inputPosition) {
        Distance dist = Centimeters.of(
                MathUtil.clamp(inputPosition.magnitude(),
                        MIN_HEIGHT_CM.magnitude(),
                        MAX_HEIGHT_CM.magnitude()));

        return dist;
    }

    /** Returns distance of encoder in centimeters. */
    public Distance getPos() {
        return Centimeters.of(encoder.getDistance());
    }

    /** Sets the position of the motor given a Distance value in centimeters. */

    public void setPosition(Distance positionCentimeters) {
        // TODO: Use Alerts subsystem to output warning message that elevator is not yet
        // homed
        if (!isHomed)
            return;
        positionCentimeters = clampToValidHeight(positionCentimeters);
        targetHeight = positionCentimeters;
        pid.setGoal(positionCentimeters.magnitude());
        double pidOutput = pid.calculate(getPos().magnitude());
        double ffOutput = elevatorFeedforward.calculate(pid.getSetpoint().velocity);
        SmartDashboard.putNumber("PID output: ", pidOutput);
        SmartDashboard.putNumber("Feedforward output: ", ffOutput);
        SmartDashboard.putNumber("Elevator voltage: ", pidOutput + ffOutput);
        leadMotor.setVoltage(pidOutput + ffOutput);
    }

    /**
     * Sets the position of the motor given a double from 0.0 to 1.0. Please note
     * that this uses the MIN_HEIGHT_CM
     * and MAX_HEIGHT_CM and scales the value accordingly. Invalid values will be
     * clamped within a valid range.
     */
    public void setPosition(double position) {
        Distance scaled = Centimeters
                .of(position * (MAX_HEIGHT_CM.magnitude() - MIN_HEIGHT_CM.magnitude()) + MIN_HEIGHT_CM.magnitude());
        setPosition(scaled);
    }

    /** Homes the elevator and resets encoder when plant triggers bottom limit switch. */
    public Command homeElevator() {
        return runOnce(
                () -> {
                    leadMotor.setVoltage(-0.1);
                }).until(
                        () -> bottomLimitSwitch.get())
                .finallyDo(
                        () -> {
                            leadMotor.set(0.0);
                            encoder.reset();
                            isHomed = true;
                        });
    }

    /**
     * Checks if the elevator position is within a certain tolerance of the goal
     * position.
     */
    public boolean isAtPosition(Distance positionCentimeters) {
        return (pid.atGoal() &&
                Math.abs(MAX_HEIGHT_CM.magnitude() - positionCentimeters.magnitude()) <= POS_TOLERANCE.magnitude());
    }

    /**
     * Checks if the elevator position is within a certain tolerance of the goal
     * position.
     */
    public boolean isAtPosition(double position) {
        return (pid.atGoal() &&
                Math.abs(MAX_HEIGHT_CM.magnitude()
                        - ((((MAX_HEIGHT_CM.magnitude() - MIN_HEIGHT_CM.magnitude())) * position))
                        + MIN_HEIGHT_CM.magnitude()) <= POS_TOLERANCE.magnitude());
    }

    /** Move to target height in centimeters. */
    public Command moveTo(Distance targetPosition) {
        return runOnce(
                () -> {
                    setPosition(targetPosition.magnitude());
                }).until(() -> isAtPosition(targetPosition));
    }

    public Command moveBy(Distance distanceToMove) {
        return runOnce(
                () -> {
                    Distance temp = Centimeters.of(MathUtil.clamp(
                            targetHeight.magnitude() + distanceToMove.magnitude(),
                            MIN_HEIGHT_CM.magnitude(),
                            MAX_HEIGHT_CM.magnitude()));
                    setPosition(temp);
                }).until(() -> isAtPosition(targetHeight));
    }

    /** Homes the elevator so accurate hieght measurements can be made. */
    public Command home() {
        return runOnce(() -> {
            leadMotor.set(-0.1);
        })
                .until(() -> bottomLimitSwitch.get())
                .finallyDo(() -> {
                    stop();
                    encoder.reset();
                });
    }

    public Command holdPosition() {
        return runOnce(() -> {
            setPosition(targetHeight);
        });
    }

}
