package frc.robot.subsystems;

public enum WristState {
    IDLE(0),
    L1(0),
    L3(0),
    L4(0),
    KNOCK(0);

    double angle;
    
    private WristState(double angle ) {
        this.angle = angle;
    }

    public double getangle() {
        return angle;
    }
}
