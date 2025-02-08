package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Wrist extends SubsystemBase {
    private final TalonFX motor;

    public Wrist() {
      motor = new TalonFX( 1);
    }

    public Command changeState(WristState state) {
        return Commands.run(() -> {
            double targetAngle = state.getangle();
            motor.set(targetAngle);
        });
    }

    @Override
    public void periodic() {

    }
}