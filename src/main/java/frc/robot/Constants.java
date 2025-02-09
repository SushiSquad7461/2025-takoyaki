package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.derive;

import javax.sound.sampled.Port;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import frc.util.control.PIDConfig;
import frc.util.motor.MotorConfig;

public class Constants {
    public static boolean TUNING_MODE;

    public static class Ports {
        public static final int PIVOT_MOTOR_ID = 0;
        public static final int ROLLER_MOTOR_ID = 0;
        public static final int BEAM_BREAK_PORT = 0; 
        public static final int ELEVATOR_LEFT_ID = 0;
        public static final int ELEVATOR_RIGHT_ID = 0;

    }

    public static class CustomUnits {
        public static final AngleUnit TalonEncoderCounts = derive(Rotations).splitInto(2048).named("Talon Encoder Counts").make();
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
            Ports.ELEVATOR_LEFT_ID,
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

    public static final class CoralManipulator {
        public static final double PIVOT_GEAR_RATIO = 0.0;
        
        // motion and position control TODO: sam pls check im using these correctly        
        public static final Measure<AngleUnit> MIN_ANGLE = Degrees.of(0);
        public static final Measure<AngleUnit> MAX_ANGLE = Degrees.of(0);
        public static final Measure<AngleUnit> ANGLE_TOLERANCE = Degrees.of(0);
        
        // feedforward and pid constants
        public static final double kG = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        
        public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
            Ports.PIVOT_MOTOR_ID,
            0,
            false,
            PIDConfig.getPid(kP, kI, kD),
            MotorConfig.Mode.BRAKE
        );
        
        public static final MotorConfig ROLLER_CONFIG = new MotorConfig(
            Ports.ROLLER_MOTOR_ID,
            0,
            false,
            MotorConfig.Mode.BRAKE
        );
        
        public static final ArmFeedforward PIVOT_FEEDFORWARD = new ArmFeedforward(
            0,
            0,
            0, 
            0
        );
        
        // for motion magic, TODO: set
        public static final double MOTION_MAGIC_VELOCITY = 0;
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_JERK = 0;
        
        // roller speeds for diff states
        public static final double INTAKE_SPEED = 0;
        public static final double SCORE_SPEED = 0;
        public static final double HOLD_SPEED = 0;
    }


}
