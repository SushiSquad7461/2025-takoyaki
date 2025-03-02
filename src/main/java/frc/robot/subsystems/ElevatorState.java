package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;

public enum ElevatorState {
    //TODO: set these height values
    IDLE(Inches.of(0)),
    L1(Inches.of(2)),
    L2(Inches.of(7)),
    L3(Inches.of(15)),
    L4(Inches.of(27.5)),       
    L3_KNOCK(Inches.of(17));         

    final Distance position;
    final double targetMotorRotations;
    
    private ElevatorState(Distance position) {
        this.position = position;
        this.targetMotorRotations = Elevator.heightToMotor(position).in(Rotations);
    }
}