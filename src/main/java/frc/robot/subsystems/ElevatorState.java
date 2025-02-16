package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public enum ElevatorState {
    //TODO: set these height values
    IDLE(Inches.of(0)),
    L1(Inches.of(0)),
    L2(Inches.of(35)),
    L3(Inches.of(47)),
    L4(Inches.of(50)),       
    KNOCK(Inches.of(37));         

    Distance position;
    
    private ElevatorState(Distance position) {
        this.position = position;
    }

    public Distance getPos() {
        return position;
    }
}