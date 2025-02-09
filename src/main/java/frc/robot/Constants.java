package frc.robot;

import frc.util.control.PIDConfig;
import frc.util.motor.MotorConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
    public static boolean TUNING_MODE;

    public static class CustomUnits {
        public static final AngleUnit TalonEncoderCounts = derive(Rotations).splitInto(2048).named("Talon Encoder Counts").symbol("TEC").make();
        public static final PerUnit<DistanceUnit, AngleUnit> MetersPerRotation = Meters.per(Rotations);
    }

    public static class Elevator {
        //creating config for software limit switch bec sushi lib doesnt handle
        //TODO: use sysid
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    
        public static final MotorConfig ELEVATOR_LEFT = new MotorConfig(
            0,
            0,
            true,
            PIDConfig.getPid(0.0),
            MotorConfig.Mode.BRAKE);

        public static final MotorConfig ELEVATOR_RIGHT = new MotorConfig(
            0,
            0,
            false,
            PIDConfig.getPid(0.0, 0.0, 0.0),
            MotorConfig.Mode.BRAKE);

        // TODO: set all of these values
        public static final Angle MAX_ERROR = Degrees.of(1.0);

        public static final int LIMIT_SWITCH_PORT = 0; 

        public static final Distance MAX_HEIGHT = Inches.of(45.0); 
        public static final Measure<? extends PerUnit<DistanceUnit, AngleUnit>> ELEVATOR_EXTENSION_PER_MOTOR_ANGLE = CustomUnits.MetersPerRotation.of(0);
        public static final Dimensionless GEAR_RATIO = Rotations.of(0).div(Rotations.of(1)); // output over input
        public static final Angle MOTOR_MAX_HEIGHT = (Angle) MAX_HEIGHT.div(ELEVATOR_EXTENSION_PER_MOTOR_ANGLE).div(GEAR_RATIO);
    }

    public static class CoralIntake {
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
}
