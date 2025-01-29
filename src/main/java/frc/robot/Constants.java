package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import frc.util.Motor.MotorConfig;
import frc.util.Control.PIDConfig;

public class Constants {
    public static boolean TUNING_MODE;

    public static class Ports {
        public static final CANBus CANIVORE_NAME = null;

    }

    public static class Swerve {
        public static final boolean REDUCE_SPEED = false;
        public static final double LOW_SPEED = 0;
        public static final double LOW_ROT = 0;
    }

    public static class Elevator {
        public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
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
        public static final double G_DOWN = 0;
        public static final double G_UP = 0;
        public static final Angle MAX_ERROR = Degrees.of(1.0);

        public static final int LIMIT_SWITCH_PORT = 0; 

        public static final double MAX_HEIGHT_INCHES = 45.0; 
        public static final double INCHES_PER_ROTATION = 1.0; 
        public static final double GEAR_RATIO = 1.0; 
        
        // calc max height in motor rotations
        public static final double MAX_HEIGHT = (MAX_HEIGHT_INCHES / INCHES_PER_ROTATION) * GEAR_RATIO * 2048; //TODO: krakens r 2048 right?
    }
}
