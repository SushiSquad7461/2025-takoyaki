package frc.robot;

import com.ctre.phoenix6.CANBus;

import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.Motor.MotorConfig;

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

    public static class Elevator {
        public static final MotorConfig ELEVATOR_LEFT = new MotorConfig(
            29,
            10,
            true,
            PIDConfig.getPid(0.12),
            MotorConfig.Mode.BRAKE);

        public static final MotorConfig ELEVATOR_RIGHT = new MotorConfig(
            30,
            10,
            true,
            PIDConfig.getPid(0.12, 0.0, 0.0),
            MotorConfig.Mode.BRAKE);

        // TODO: set all of these values
        public static final double G_DOWN = 0;
        public static final double G_UP = 0;

        public static final double MAX_ERROR = 1.0;

        public static final int LIMIT_SWITCH_PORT = 0; 

        public static final double MAX_HEIGHT_INCHES = 45.0; 
        public static final double INCHES_PER_ROTATION = 1.0; 
        public static final double GEAR_RATIO = 1.0; 
        
        // calc max height in motor rotations
        public static final double MAX_HEIGHT = (MAX_HEIGHT_INCHES / INCHES_PER_ROTATION) * GEAR_RATIO * 2048; //TODO: krakens r 2048 right?
        }




    
}
