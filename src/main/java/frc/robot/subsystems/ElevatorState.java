package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public enum ElevatorState {
    IDLE(Inches.of(0)),
    L1(Inches.of(20)),
    L2(Inches.of(35)),
    L3(Inches.of(47)),
    L4(Inches.of(50));         

    Distance position;
    
    private ElevatorState(Distance position) {
        this.position = position;
    }

    public Distance getPos() {
        return position;
    }
}