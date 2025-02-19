package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Direction;
import frc.robot.util.control.nt.PIDTuning;
import frc.robot.util.control.nt.TunableNumber;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

public class Intake extends SubsystemBase {
    
    private final TalonFX pivotMotor;
    private final TalonFX wheelMotor;
    private TunableNumber pivotPos;
    private PIDTuning pivotPID;
    private SysIdRoutine routine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    public Intake() {
        super();

        pivotMotor = Constants.AlgaeIntake.PIVOT_CONFIG.createTalon();
        wheelMotor = Constants.AlgaeIntake.INTAKE_CONFIG.createTalon();
        pivotPos = new TunableNumber("Intake Pos", Constants.AlgaeIntake.RAISED_POS.in(Degrees), Constants.TUNING_MODE);
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

    //Commands for the intake 
    private Command lowerIntake() {
        return changePivotPos(Constants.AlgaeIntake.LOWERED_POS);
    }

    private Command raiseIntake() {
        return changePivotPos(Constants.AlgaeIntake.RAISED_POS)
            .andThen(() -> pivotMotor.set(0.1))
            .until(this::intakeAtTop)
            .andThen(() -> pivotMotor.setPosition(Constants.AlgaeIntake.RAISED_POS));
    }

    private Command runIntake() {
        return runOnce(() -> wheelMotor.set(Constants.AlgaeIntake.INTAKE_SPEED));
    }

    private boolean intakeAtTop() {
        return currentSpike();
    }
    
    private boolean currentSpike() {
        return (pivotMotor.getPosition().getValue().compareTo(Constants.AlgaeIntake.MAX_SPIKE_HEIGHT) < 0 &&
            pivotMotor.getSupplyCurrent().getValue().compareTo(Constants.AlgaeIntake.CURRENT_LIMIT) > 0);
    }

    private Command reverseIntake() {
        return runOnce(() -> wheelMotor.set(-1 * Constants.AlgaeIntake.INTAKE_SPEED));
    }

    private Command stopIntake() {
        return runOnce(() -> wheelMotor.set(0.0));
    }

    //Changing the position of the intake
    private Command changePivotPos(Angle position) {
        pivotMotor.setControl(
            new PositionDutyCycle(position.div(Constants.AlgaeIntake.INTAKE_GEAR_RATIO))
        );

        return run(() -> {
            pivotPos.setDefault(position.in(Degrees));
        }).until(() -> getError(position).abs(Degrees) < Constants.AlgaeIntake.MAX_ERROR.in(Degrees));
    }

    //Changes the state of the intake
    public Command changeState(IntakeState newState) {
        //Will check to see if intake is up, if it, lower intake, else, raise intake
        Command pivotCommand = newState.intakeExtended ? lowerIntake() : raiseIntake();
        //Checks to see if state reverses intake, if it does then reverse intake, if not run intake
        Command intakeCommand = newState.intakeExtended ? (newState.direction == Direction.REVERSED ? reverseIntake() : runIntake())
                //Will stop the intake if not extended
                : stopIntake();
        return pivotCommand.andThen(intakeCommand);
    }

    public void periodic() {
        pivotPID.updatePID(pivotMotor);
    }
}