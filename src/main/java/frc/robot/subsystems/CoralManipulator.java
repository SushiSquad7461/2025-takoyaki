package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private final BooleanPublisher coralInManipulatorPub;
    private final BooleanPublisher coralInputtedPub;

    public CoralManipulator() {
        rollerMotor = Constants.CoralManipulator.ROLLER_CONFIG.createTalon();
        beambreak = new DigitalInput(Constants.Ports.BEAM_BREAK_PORT);
        elevBeambreak = new DigitalInput(Constants.Ports.ELEV_BEAM_BREAK_PORT);

        // network table setup
        manipulatorTable = NetworkTableInstance.getDefault().getTable("Manipulator");
        coralInManipulatorPub = manipulatorTable.getBooleanTopic("BeamBreak").publish();
        coralInputtedPub = manipulatorTable.getBooleanTopic("Elevator BeamBreak").publish();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command changeState(ManipulatorState state) {
        return switch(state) {
            case IDLE -> stopRollers();
            case INTAKE -> runRollers(state.getRollerSpeed())
                .andThen(Commands.waitUntil(this::coralInputted))
                .andThen(Commands.waitUntil(this::hasCoral))
                .andThen(stopRollers())
                .unless(this::hasCoral);
            case KNOCK -> runRollers(state.getRollerSpeed()).unless(() -> !hasCoral());
            case SCORE -> runRollers(state.getRollerSpeed())
                .andThen(Commands.waitUntil(() -> !hasCoral()))
                .andThen(Commands.waitSeconds(0.5))
                .unless(() -> !hasCoral());
        };
    }

    public Command runRollers(double speed) {
        // return runOnce(() -> rollerMotor.setControl(veloDutyCycle.withVelocity(speed)));
        return runOnce(() -> rollerMotor.set(speed));

    }

    public Command stopRollers() {
        return runRollers(0);
    }

    public boolean hasCoral() {
        return elevBeambreak.get() && !beambreak.get(); // get() returns true when circuit is closed
    }


    public boolean coralInputted(){
        return !elevBeambreak.get();
    }

    @Override
    public void periodic() {
        coralInManipulatorPub.set(!beambreak.get());
        coralInputtedPub.set(!elevBeambreak.get());
    }

}
