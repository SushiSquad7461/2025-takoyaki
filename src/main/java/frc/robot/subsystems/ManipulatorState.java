package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

public enum ManipulatorState {
    IDLE(Degrees.of(0), MetersPerSecond.of(0)),
    L1(Degrees.of(45), Constants.CoralManipulator.SCORE_SPEED),
    L3(Degrees.of(90), Constants.CoralManipulator.SCORE_SPEED),
    L4(Degrees.of(135), Constants.CoralManipulator.SCORE_SPEED),
    KNOCK(Degrees.of(20), Constants.CoralManipulator.SCORE_SPEED),
    INTAKE(Degrees.of(0), Constants.CoralManipulator.INTAKE_SPEED);

    private final Angle angle;
    private final LinearVelocity speed;

    private ManipulatorState(Angle angle, LinearVelocity speed) {
        this.angle = angle;
        this.speed = speed;
    }
    
    public Angle getAngle() {
        return angle;
    }

    public LinearVelocity getRollerSpeed() {
        return speed;
    }

}
