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

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static boolean TUNING_MODE;

    public static class Ports
    {
        public static final CANBus CANIVORE_NAME = null;

    }

    public static class Swerve
    {
        public static final boolean REDUCE_SPEED = false;
        public static final double LOW_SPEED = 0;
        public static final double LOW_ROT = 0;
    }

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
}
