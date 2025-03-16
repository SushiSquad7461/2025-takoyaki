package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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

    private final DCMotor rollerMotorModel = DCMotor.getKrakenX60(1);
    private final DCMotorSim rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(rollerMotorModel, 1.0, 1),
        rollerMotorModel);
    private final TalonFXSimState rollerMotorSim;
    private double simCurrentDrawAmps = 0;

    public CoralManipulator() {
        rollerMotor = new TalonFX(Constants.Ports.CORAL_ROLLER_MOTOR_ID);
        rollerMotor.getConfigurator().apply(Constants.CoralManipulator.ROLLER_CONFIG);
        beambreak = new DigitalInput(Constants.Ports.BEAM_BREAK_PORT);
        elevBeambreak = new DigitalInput(Constants.Ports.ELEV_BEAM_BREAK_PORT); //plugged in different from before, so swapped values

        // network table setup
        manipulatorTable = NetworkTableInstance.getDefault().getTable("Manipulator");
        coralInManipulatorPub = manipulatorTable.getBooleanTopic("BeamBreak").publish();
        coralInputtedPub = manipulatorTable.getBooleanTopic("Elevator BeamBreak").publish();

        rollerMotorSim = rollerMotor.getSimState();
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
            case SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4 -> runRollers(state.getRollerSpeed())
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

    @Override
    public void simulationPeriodic() {
        var supplyVoltage = RobotController.getBatteryVoltage();
        rollerMotorSim.setSupplyVoltage(supplyVoltage);
        rollerSim.setInputVoltage(rollerMotorSim.getMotorVoltage());
        rollerSim.update(Constants.LOOP_TIME_SECONDS);
        rollerMotorSim.setRawRotorPosition(rollerSim.getAngularPosition());
        rollerMotorSim.setRotorVelocity(Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()));
        simCurrentDrawAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    }

    public double getSimulatedCurrentDrawAmps() {
        return simCurrentDrawAmps;
    }
}
