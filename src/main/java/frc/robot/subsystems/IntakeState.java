package frc.robot.subsystems;


import frc.robot.Direction;

public enum IntakeState {
    //All Intake states 
    IDLE(false, Direction.OFF),
    INTAKE(true, Direction.RUNNING),
    REVERSE(true, Direction.REVERSED),
    CARRYING(true, Direction.OFF);
    //TODO: Add more intake states 

    public boolean intakeExtended;
    public Direction direction;

    private IntakeState(boolean extended, Direction direction) {
        this.intakeExtended = extended;
        this.direction = direction;
    }
}  