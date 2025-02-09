package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;

public enum ManipulatorState {
    IDLE(Degrees.of(0)),
    L1(Degrees.of(45)),
    L3(Degrees.of(90)),
    L4(Degrees.of(135)),
    KNOCK(Degrees.of(20)),
    INTAKE(Degrees.of(0));

    private final Measure<AngleUnit> angle;
    
    private ManipulatorState(Measure<AngleUnit> angle) {
        this.angle = angle;
    }
    
    public Measure<AngleUnit> getAngle() {
        return angle;
    }
}
