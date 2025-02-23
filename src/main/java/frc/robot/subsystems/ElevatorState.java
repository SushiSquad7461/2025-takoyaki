package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public enum ElevatorState {
    //TODO: set these height values
    IDLE(Inches.of(0)),
    L1(Inches.of(2)),
    L2(Inches.of(5)),
    L3(Inches.of(13)),
    L4(Inches.of(24)),       
    L3_KNOCK(Inches.of(17));         

    Distance position;
    
    private ElevatorState(Distance position) {
        this.position = position;
    }

    public Distance getPos() {
        return position;
    }
}