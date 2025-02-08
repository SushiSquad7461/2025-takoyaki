package frc.robot;

import frc.util.Control.PIDConfig;
import frc.util.Motor.MotorConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;

public final class Constants {
    public static final boolean TUNING_MODE = true;

    // Pivot Motor Configuration
    public static final MotorConfig PIVOT = new MotorConfig(
        0,  // CAN ID (needs to be set)
        0,  // Current limit (needs to be set)
        false,  // Inverted?
        PIDConfig.getPid(0.0, 0.0, 0.0, 0.0), // PID values (needs tuning)
        MotorConfig.Mode.BRAKE
    );

    // Gear Ratio: Motor rotations to wrist pivot rotations
    public static final double PIVOT_GEAR_RATIO = 81.0; 

    // Speed settings
    public static final double MAX_PIVOT_SPEED = 0.5; // Needs to be set

    // Pivot Angle Limits (prevents over-rotation)
    public static final double MIN_ANGLE = Units.degreesToRadians(0); // Needs to be set
    public static final double MAX_ANGLE = Units.degreesToRadians(180.0); // Needs to be set

    // Encoder Settings
    public static final int ENCODER_CHANNEL = 0; // Change if needed
    public static final double ENCODER_OFFSET = 0.0; // Offset for zero position

    // Feedforward control for precise wrist movement
    public static final ArmFeedforward PIVOT_FEEDFORWARD = new ArmFeedforward(0.0, 0.0, 0.0);

    // Tolerance for wrist angle
    public static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.0); 
}
