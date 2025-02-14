package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final DigitalInput beambreak;
    private final NetworkTable manipulatorTable;
    private final BooleanPublisher beambreakPub;
    private final DoublePublisher currentPub;

    public CoralManipulator() {
        rollerMotor = Constants.CoralManipulator.ROLLER_CONFIG.createTalon();
        beambreak = new DigitalInput(Constants.Ports.BEAM_BREAK_PORT);
        
        // network table setup
        manipulatorTable = NetworkTableInstance.getDefault().getTable("Manipulator");
        beambreakPub = manipulatorTable.getBooleanTopic("BeamBreak").publish();
        currentPub = manipulatorTable.getDoubleTopic("Current").publish();
    }

    public Command changeState(ManipulatorState state) {
        if (state == ManipulatorState.IDLE) {
            return stopRollers();
        }
        
        if (state == ManipulatorState.INTAKE) {
            // for intake, run until detect coral
            return runRollers(state.getRollerSpeed())
                .until(this::hasCoral);
        }

        // for scoring, only run if coral piece is detected
        if (state == ManipulatorState.SCORE && !hasCoral()) {
            return Commands.none();
        }

        // else, default to running rollers
        return runRollers(state.getRollerSpeed());
    }
    
    public Command runRollers(double speed) {
        return runOnce(() -> rollerMotor.set(speed));
    }
    
    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }
        
    public boolean hasCoral() {
        return !beambreak.get(); //get() returns true when circuit is closed
    }

    @Override
    public void periodic() {
        beambreakPub.set(hasCoral());
        currentPub.set(rollerMotor.getSupplyCurrent().getValueAsDouble());
    }
}