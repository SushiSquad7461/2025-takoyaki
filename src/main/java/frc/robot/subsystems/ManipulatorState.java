package frc.robot.subsystems;
import frc.robot.Constants;

    public enum ManipulatorState {
        IDLE(0),
        SCORE(Constants.CoralManipulator.SCORE_SPEED),
        KNOCK(Constants.CoralManipulator.SCORE_SPEED),
        INTAKE(Constants.CoralManipulator.INTAKE_SPEED);

        private final double speed;

        private ManipulatorState(double speed) {
            this.speed = speed;
        }

        public double getRollerSpeed() {
            return speed;
        }
    }
