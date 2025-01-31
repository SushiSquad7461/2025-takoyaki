package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import frc.util.Motor.MotorConfig;
import frc.util.Control.PIDConfig;

public class Constants {
    public static boolean TUNING_MODE;

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

        public static final Distance MAX_HEIGHT_INCHES = Inches.of(45.0); 
        public static final Distance INCHES_PER_ROTATION = Inches.of(1.0); ; 
        public static final double GEAR_RATIO = 1.0; 
        
        // calc max height in motor rotations
        public static final Dimensionless MAX_HEIGHT = MAX_HEIGHT_INCHES.div(INCHES_PER_ROTATION).times(GEAR_RATIO).times(2048); //TODO: krakens r 2048 right?
    }
}
