package frc.robot.subsystems;

    public enum ManipulatorState {
        IDLE(0),
        SCORE(0.2),
        KNOCK(0.2),
        INTAKE(0.1);

        /** Duty cycle in range [-1,1] */
        private final double speed;

        private ManipulatorState(double speed) {
            this.speed = speed;
        }

        public double getRollerSpeed() {
            return speed;
        }
    }
