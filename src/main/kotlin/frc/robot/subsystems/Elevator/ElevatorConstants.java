package frc.robot.subsystems.Elevator;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;

public final class ElevatorConstants {
    private ElevatorConstants(){};

    public static final int ELEVATOR_LEADER_MOTOR_ID = 1;
    public static final int ENCODER_LA_ID = 2;
    public static final int ENCODER_LB_ID = 3;

    public static final int[] FOLLOWER_MOTOR_IDS = {67, 67, 67};

    public static final int ENCODER_FA_ID = 5;
    public static final int ENCODER_FB_ID = 6;

    public static final Distance MIN_HEIGHT_CM = Centimeters.of(20);
    public static final Distance MAX_HEIGHT_CM = Centimeters.of(200);

    public static final LinearVelocity MAX_VELOCITY_M_S = 
        MetersPerSecond.of(3.0);
    public static final LinearAcceleration MAX_ACCELERATION_M_S_SQ = 
        MetersPerSecondPerSecond.of(2.0);

    /** Encoder pulses per centimeter */
    public static final int ENCODER_PPC = 100;
    
    public static final int BOTTOM_LIMIT_SWITCH_ID = 0;
    public static final int TOP_LIMIT_SWITCH_ID = 0;
    
    public static final double kP = 0.0; // Proportional coefficient of PID controller
    public static final double kI = 0.0; // Integral coefficient of PID controller
    public static final double kD = 0.0; // Derivative coefficient of PID controller
    public static final double kS = 0.0; 
    public static final double kG = 0.0; // Voltage required to overcome force of gravity, aka hold plant in place
    public static final double kV = 0.0;

    public static final Distance POS_TOLERANCE = Inches.of(0.0); // Tolerance for position
    
    public static final TrapezoidProfile.Constraints ELEVATOR_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_VELOCITY_M_S.in(MetersPerSecond), MAX_ACCELERATION_M_S_SQ.in(MetersPerSecondPerSecond));
}
