package frc.robot.subsystems;

    public enum ManipulatorState {
        IDLE(0),
        SCORE_L1(0.15),
        SCORE_L2(0.1),
        SCORE_L3(0.1),
        SCORE_L4(0.15),
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
