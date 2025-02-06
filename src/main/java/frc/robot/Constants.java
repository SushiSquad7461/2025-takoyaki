package frc.robot;

import frc.util.Control.PIDConfig;
import frc.util.Motor.MotorConfig;

public class Constants {
        //TODO: change constants
        public enum IntakeENums {
            //All Intake states 
            IDLE(false, Direction.OFF),
            INTAKE(true, Direction.RUNNING),
            REVERSE(true, Direction.REVERSED),
            CARRYING(true, Direction.REVERSED);
            //TODO: Add more intake states 
        
            public boolean intakeExtended;
            public Direction direction;
        
            private IntakeENums(boolean extended, Direction direction) {
                this.intakeExtended = extended;
                this.direction = direction;
            }
  }  
        public static final class Intake {
        public static final double G = 0.0; 
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
            MotorConfig.Mode.COAST);

        public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
            0,
            0,
            true,
            PIDConfig.getPid(0.0, 0.0, 0.0),
            MotorConfig.Mode.BRAKE);
        }

        public static final boolean TUNING_MODE = false;

  
}
