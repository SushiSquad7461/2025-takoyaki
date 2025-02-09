package frc.robot;

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
import frc.util.control.PIDConfig;
import frc.util.motor.MotorConfig;

public class Constants {
    public static final boolean TUNING_MODE = false;

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
       
    //TODO: change constants
    public static class AlgaeIntake {
        public enum IntakeState {
            //All Intake states 
            IDLE(false, Direction.OFF),
            INTAKE(true, Direction.RUNNING),
            REVERSE(true, Direction.REVERSED),
            CARRYING(true, Direction.REVERSED);
            //TODO: Add more intake states 
        
            public boolean intakeExtended;
            public Direction direction;
        
            private IntakeState(boolean extended, Direction direction) {
                this.intakeExtended = extended;
                this.direction = direction;
            }
        }  
        public static final class Intake {
        public static final double G = 0.0;
        public static final double armFeedForward = 0.0; 
        public static final int ENCODER_CHANNEL = 0;
        public static final double ENCODER_ANGLE_OFFSET = 0; 
        public static final double INTAKE_GEAR_RATIO = 0.0;

        public static final double INTAKE_SPEED = 0.0;

        public static final double ERROR_LIMIT = 0.0;
        public static final double MAX_ERROR = 0.0;

        public static final double RAISED_POS = 0;
        public static final double LOWERED_POS = 0; 

        public static final MotorConfig INTAKE_CONFIG = new MotorConfig(
            0,
            0,
            true, 
            null, 
            MotorConfig.Mode.COAST);

        public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
            0,
            0,
            true,
            PIDConfig.getPid(0.0, 0.0, 0.0),
            MotorConfig.Mode.BRAKE);
        };
    }
}
