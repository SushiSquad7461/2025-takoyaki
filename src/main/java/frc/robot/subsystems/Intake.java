package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;

public class Intake extends SubsystemBase {
    
    private final TalonFX pivotMotor;
    /** getValueAsDouble returns in Amps */
    private final StatusSignal<Current> pivotMotorCurrent;
    private final TalonFX wheelMotor;
    private SysIdRoutine routine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public Intake() {
        pivotMotor = new TalonFX(Constants.Ports.INTAKE_PIVOT_ID);
        pivotMotor.getConfigurator().apply(Constants.AlgaeIntake.PIVOT_CONFIG);
        pivotMotorCurrent = pivotMotor.getSupplyCurrent();
        wheelMotor = new TalonFX(Constants.Ports.ALGAE_INTAKE_ROLLER_ID);
        wheelMotor.getConfigurator().apply(Constants.AlgaeIntake.INTAKE_CONFIG);
        
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
                    pivotMotor.setPosition(Constants.AlgaeIntake.INTAKE_ANGLE);
                    pivotMotor.setControl(positionDutyCycle.withPosition(Constants.AlgaeIntake.INTAKE_ANGLE));
                });
    }

    private Command runIntake() {
        return runOnce(() -> wheelMotor.set(Constants.AlgaeIntake.INTAKE_SPEED));
    }
    
    private boolean currentSpikeUp() {
        return pivotMotorCurrent.getValueAsDouble() > Constants.AlgaeIntake.CURRENT_SPIKE_LIMIT_UP_AMPS;
    }

    private boolean currentSpikeDown() {
        return pivotMotorCurrent.getValueAsDouble() > Constants.AlgaeIntake.CURRENT_SPIKE_LIMIT_DOWN_AMPS;
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
        pivotMotorCurrent.refresh();
    }
}