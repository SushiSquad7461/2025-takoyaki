package frc.robot.subsystems;

import frc.robot.util.Direction;

public enum IntakeState {
    //All Intake states 
    IDLE(false, Direction.OFF), //retracted and stopped
    SCORE(false, Direction.REVERSED), //score possibility 1, retracted and reversed
    INTAKE(true, Direction.RUNNING), //extended and running forward
    REVERSE(true, Direction.REVERSED), //score possibility 2, extended and running backward
    CARRYING(true, Direction.REVERSED); //extended and holding algae (reversed)

    //TODO: Add more intake states 

    public boolean intakeExtended;
    public Direction direction;

    private IntakeState(boolean extended, Direction direction) {
        this.intakeExtended = extended;
        this.direction = direction;
    }
}  

