package frc.robot;

import frc.util.Control.PIDConfig;
import frc.util.Motor.MotorConfig;

public class Constants {
        //TODO: change constants
        public static final class Intake {
        public static final double G = 0.25; // retune
        public static final int ENCODER_CHANNEL = 1;
        public static final double ENCODER_ANGLE_OFFSET = 71; // 60
        public static final double INTAKE_GEAR_RATIO = 21.701;

        public static final double INTAKE_SPEED = 0.5;

        public static final double ERROR_LIMIT = 1.0;
        public static final double MAX_ERROR = 4.0;

        public static final double RAISED_POS = 110;
        public static final double LOWERED_POS = -12; // -26

        public static final MotorConfig INTAKE_CONFIG = new MotorConfig(
            21,
            40, // set later
            true, // spin motor
            MotorConfig.Mode.COAST);

        public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
            22,
            20,
            true,
            PIDConfig.getPid(0.009, 0.01, 0.0), // p:0.012d:0.6
            MotorConfig.Mode.BRAKE);
        }

        public static final boolean TUNING_MODE = false;

  
}
