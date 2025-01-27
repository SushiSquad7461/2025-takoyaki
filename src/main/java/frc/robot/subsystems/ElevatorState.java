package frc.robot.subsystems;

public enum ElevatorState {
    IDLE(0),
    L1(20),
    L2(35),
    L3(47),
    L4(50);         

    double position;
    
    private ElevatorState(double position) {
        this.position = position;
    }

    public double getPos() {
        return position;
    }
}