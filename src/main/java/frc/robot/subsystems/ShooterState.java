package frc.robot.subsystems;

import frc.robot.Direction;

public enum ShooterState {
    IDLE(false,0.0, Direction.OFF),
    INTAKE(true,0.0,Direction.FORWARD),
    HOLDING(true,0.0,Direction.OFF),
    SHOOT_PROCESSOR(false,0.0,Direction.REVERSE),
    SHOOT_BARGE(false,0.0,Direction.REVERSE);

    private boolean extended;
    private double speed;
    private Direction direction;

    private ShooterState(boolean extended, double speed, Direction direction){
        this.extended = extended;
        this.speed = speed;
        this.direction = direction;
    }
}
