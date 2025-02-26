package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final DigitalInput beambreak;
    private final DigitalInput elevBeambreak;
    public SysIdRoutine routine;

    private final NetworkTable manipulatorTable;
    private final BooleanPublisher beambreakPub;
    private final BooleanPublisher elevBeambreakPub;
    private final DoublePublisher currentPub;

    public CoralManipulator() {
        rollerMotor = Constants.CoralManipulator.ROLLER_CONFIG.createTalon();
        beambreak = new DigitalInput(Constants.Ports.BEAM_BREAK_PORT);
        elevBeambreak = new DigitalInput(Constants.Ports.ELEV_BEAM_BREAK_PORT);

        // network table setup
        manipulatorTable = NetworkTableInstance.getDefault().getTable("Manipulator");
        beambreakPub = manipulatorTable.getBooleanTopic("BeamBreak").publish();
        elevBeambreakPub = manipulatorTable.getBooleanTopic("Elevator BeamBreak").publish();
        currentPub = manipulatorTable.getDoubleTopic("Current").publish();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command changeState(ManipulatorState state) {
        if (state == ManipulatorState.IDLE) {
            return stopRollers();
        }

        if (state == ManipulatorState.INTAKE) {
            // for intake, run until detect coral
            return runRollers(state.getRollerSpeed())
                .andThen(Commands.waitUntil(this::coralInputted))
                .andThen(Commands.waitUntil(this::hasCoral))
                .andThen(Commands.waitTime(Seconds.of(0.25)))
                .andThen(stopRollers())
                .unless(this::hasCoral);
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
        return elevBeambreak.get() && !beambreak.get(); // get() returns true when circuit is closed
    }


    public boolean coralInputted(){
        return !elevBeambreak.get();
    }

    @Override
    public void periodic() {
        beambreakPub.set(!beambreak.get());
        elevBeambreakPub.set(!elevBeambreak.get());
        currentPub.set(rollerMotor.getSupplyCurrent().getValue().in(Amps));
    }

}
