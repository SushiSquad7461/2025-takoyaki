package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Direction;
import frc.robot.util.control.nt.PIDTuning;
import frc.robot.util.control.nt.TunableNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class Intake extends SubsystemBase {
    
    private final TalonFX pivotMotor;
    private final TalonFX wheelMotor;
    private TunableNumber pivotPos;
    private PIDTuning pivotPID;

    public Intake() {
        super();

        pivotMotor = Constants.AlgaeIntake.PIVOT_CONFIG.createTalon();
        wheelMotor = Constants.AlgaeIntake.INTAKE_CONFIG.createTalon();
        pivotPos = new TunableNumber("Intake Pos", Constants.AlgaeIntake.RAISED_POS.magnitude(), Constants.TUNING_MODE);
        pivotPID = Constants.AlgaeIntake.PIVOT_CONFIG.genPIDTuning("Pivot Intake", Constants.TUNING_MODE);

         routine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,        // Use default ramp rate (1 V/s)
        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
        null,        // Use default timeout (10 s)
                     // Log state with Phoenix SignalLogger class
        (state) -> SignalLogger.writeString("state", state.toString())
     ),
     new SysIdRoutine.Mechanism(
      (volts) -> 
      {
        pivotMotor.setControl(m_voltReq.withOutput(volts.in(Volts))); 
        wheelMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
      }
      ,
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

    //retrieving position and other related values
    private void resetToAbsolutePosition() {
        pivotMotor.setPosition(getAbsolutePosition());
    }

    private double getAbsolutePosition() {
        return MathUtil.inputModulus(getPosition(), -180, 180);
    }

    private double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    private double getError(Angle setpoint) {
        return Math.abs(getPosition() - setpoint.magnitude());
    }

    private double getAbsoluteError() {
        return Math.abs(getPosition() - getAbsolutePosition());
    }


    //Commands for the intake 
    private Command lowerIntake() {
        return changePivotPos(Constants.AlgaeIntake.LOWERED_POS);
    }

    private Command raiseIntake() {
        return changePivotPos(Constants.AlgaeIntake.RAISED_POS);
    }

    private Command runIntake() {
        return runOnce(() -> wheelMotor.set(Constants.AlgaeIntake.INTAKE_SPEED));
    }

    private Command reverseIntake() {
        return runOnce(() -> wheelMotor.set(-1 * Constants.AlgaeIntake.INTAKE_SPEED));
    }

    private Command stopIntake() {
        return runOnce(() -> wheelMotor.set(0.0));
    }

    //Changing the position of the intake
    private Command changePivotPos(Angle position) {
        if (getAbsoluteError() > Constants.AlgaeIntake.ERROR_LIMIT) {
            resetToAbsolutePosition();
        }

        pivotMotor.setControl(
            new PositionDutyCycle(position.div(Constants.AlgaeIntake.INTAKE_GEAR_RATIO))
        );

        return run(() -> {
            pivotPos.setDefault(position.magnitude());
        }).until(() -> getError(position) < Constants.AlgaeIntake.MAX_ERROR);
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
        if (getAbsoluteError() > Constants.AlgaeIntake.ERROR_LIMIT) {
            resetToAbsolutePosition();
        }

        pivotPID.updatePID(pivotMotor);
    }
}
