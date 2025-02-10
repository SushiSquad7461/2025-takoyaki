package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public enum ManipulatorState {
    IDLE(Degrees.of(0), 0),
    L1(Degrees.of(45), Constants.CoralManipulator.SCORE_SPEED),
    L3(Degrees.of(90), Constants.CoralManipulator.SCORE_SPEED),
    L4(Degrees.of(135), Constants.CoralManipulator.SCORE_SPEED),
    KNOCK(Degrees.of(20), Constants.CoralManipulator.SCORE_SPEED),
    INTAKE(Degrees.of(0), Constants.CoralManipulator.INTAKE_SPEED);

    private final Angle angle;
    private final double speed;

    private ManipulatorState(Angle angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }
    
    public Angle getAngle() {
        return angle;
    }

    public double getRollerSpeed() {
        return speed;
    }

}
