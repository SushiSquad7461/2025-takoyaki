package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.control.nt.PIDTuning;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;

public class Intake extends SubsystemBase {
    
    private final TalonFX pivotMotor;
    private final TalonFX wheelMotor;
    private PIDTuning pivotPID;
    private SysIdRoutine routine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Algae Intake");
    private final DoublePublisher posPub = intakeTable.getDoubleTopic("Position").publish();
    private final DoublePublisher currentPub = intakeTable.getDoubleTopic("Current").publish();
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public Intake() {
        super();

        pivotMotor = Constants.AlgaeIntake.PIVOT_CONFIG.createTalon();
        wheelMotor = Constants.AlgaeIntake.INTAKE_CONFIG.createTalon();
        pivotPID = Constants.AlgaeIntake.PIVOT_CONFIG.genPIDTuning("Pivot Intake", Constants.TUNING_MODE);
        
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    pivotMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
                },
                null,
                this
            )
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
    
    private Angle getPosition() {
        return pivotMotor.getPosition().getValue();
    }

    private Angle getError(Angle setpoint) {
        return getPosition().minus(setpoint);
    }

    public Command reset(boolean up) {
        return up
            ? runOnce(() -> pivotMotor.set(-0.1))
                .andThen(Commands.waitUntil(this::currentSpikeUp))
                .andThen(() -> {
                    pivotMotor.stopMotor();
                    pivotMotor.setPosition(0);
                    pivotMotor.setControl(positionDutyCycle.withPosition(0));
                })
            : runOnce(() -> pivotMotor.set(0.1))
                .andThen(Commands.waitUntil(this::currentSpikeDown))
                .andThen(() -> {
                    pivotMotor.stopMotor();
                    //TODO: MAKE A VAR FOR THE DEGREES
                    pivotMotor.setPosition(Degrees.of(56.85).times(Constants.AlgaeIntake.INTAKE_GEAR_RATIO));
                    pivotMotor.setControl(positionDutyCycle.withPosition(Degrees.of(56.85).times(Constants.AlgaeIntake.INTAKE_GEAR_RATIO)));
                });
    }

    private Command runIntake() {
        return runOnce(() -> wheelMotor.set(Constants.AlgaeIntake.INTAKE_SPEED));
    }
    
    private boolean currentSpikeUp() {
        return (pivotMotor.getSupplyCurrent().getValue().compareTo(Constants.AlgaeIntake.CURRENT_SPIKE_LIMIT_UP) > 0);
    }

    private boolean currentSpikeDown() {
        return (pivotMotor.getSupplyCurrent().getValue().compareTo(Constants.AlgaeIntake.CURRENT_SPIKE_LIMIT_DOWN) > 0);
    }

    private Command reverseIntake() {
        return runOnce(() -> wheelMotor.set(-1 * Constants.AlgaeIntake.INTAKE_SPEED));
    }

    private Command stopIntake() {
        return runOnce(() -> wheelMotor.set(0.0));
    }

    private BooleanSupplier intakeInPosition(Angle position) {
        return () -> getError(position).abs(Degrees) < Constants.AlgaeIntake.MAX_ERROR.in(Degrees);
    }

    //Changing the position of the intake
    private Command changePivotPos(Angle position) {
        return runOnce(() -> pivotMotor.setControl(positionDutyCycle.withPosition(position)))
            .andThen(Commands.waitUntil(intakeInPosition(position)));
    }

    //Changes the state of the intake
    public Command changeState(IntakeState newState) {
        //Will check to see if intake is up, if it, lower intake, else, raise intake
        final Command pivotCommand = newState.intakeExtended
            ? changePivotPos(Constants.AlgaeIntake.LOWERED_POS).andThen(reset(false))
            : changePivotPos(Constants.AlgaeIntake.RAISED_POS).andThen(reset(true));
        //Checks to see if state reverses intake, if it does then reverse intake, if not run intake
        final Command intakeCommand = switch(newState.direction) {
            case REVERSE -> reverseIntake();
            case FORWARD -> runIntake();
            case OFF -> stopIntake();
        };
        return pivotCommand.andThen(intakeCommand);
    }

    public void periodic() {
        posPub.set(pivotMotor.getPosition().getValue().in(Rotations));
        currentPub.set(pivotMotor.getSupplyCurrent().getValue().in(Amps));
        pivotPID.updatePID(pivotMotor);
    }
}