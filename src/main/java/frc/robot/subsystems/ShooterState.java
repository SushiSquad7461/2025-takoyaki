package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Direction;

public enum ShooterState {
    IDLE(false,0.0, Direction.OFF),
    INTAKE(true,0.0,Direction.FORWARD),
    HOLDING(true,0.0,Direction.OFF),
    SHOOT_PROCESSOR(false,0.0,Direction.REVERSE, Constants.AlgaeShooter.PROCESSOR_POS),
    SHOOT_BARGE(false,0.0,Direction.REVERSE, Constants.AlgaeShooter.BARGE_POS);

    public boolean intaking;
    public double speed;
    public Direction direction;
    public Angle shootPos;


    private ShooterState(boolean intaking, double speed, Direction direction){
        this.intaking = intaking;
        this.speed = speed;
        this.direction = direction;
    }
    private ShooterState(boolean intaking, double speed, Direction direction, Angle shootPos){
        this.intaking = intaking;
        this.speed = speed;
        this.direction = direction;
        this.shootPos = shootPos;
    }
}
