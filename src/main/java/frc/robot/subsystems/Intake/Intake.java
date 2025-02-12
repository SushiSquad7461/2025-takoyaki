package frc.robot.subsystems.Intake;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Direction;

import java.util.function.BooleanSupplier;

import frc.util.motor.MotorHelper;
import frc.util.control.nt.PIDTuning;
import frc.util.control.nt.TunableNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.ctre.phoenix6.CANBus;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;



abstract public class Intake extends SubsystemBase {
    
    private final TalonFX pivotMotor;
    private final TalonFX wheelMotor;
    private final ArmFeedforward intakeFeedforward;
    private TunableNumber pivotPos;
    private PIDTuning pivotPID;


    public Intake() {
        super();

        intakeFeedforward = new ArmFeedforward(0.0, Constants.AlgaeIntake.G, 0.0);

        pivotMotor = Constants.AlgaeIntake.PIVOT_CONFIG.createTalon();
        wheelMotor = Constants.AlgaeIntake.INTAKE_CONFIG.createTalon();


  //      MotorHelper.setDegreeConversionFactor(pivotMotor, Constants.AlgaeIntake.INTAKE_GEAR_RATIO);

       pivotPos = new TunableNumber("Intake Pos", Constants.AlgaeIntake.RAISED_POS, Constants.TUNING_MODE);
       pivotPID = Constants.AlgaeIntake.PIVOT_CONFIG.genPIDTuning("Pivot Intake", Constants.TUNING_MODE);
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

    private BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < Constants.AlgaeIntake.MAX_ERROR);
    }

    private double getError(double setpoint) {
        return Math.abs(getPosition() - setpoint);
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

 

    //Changing the position for the intake
    private Command changePivotPos(double position) {
        if (getAbsoluteError() > Constants.AlgaeIntake.ERROR_LIMIT) {
            resetToAbsolutePosition();
        }

        pivotMotor.setControl(
            new PositionDutyCycle(199.5).div(3.75)
        );

        return run(() -> {
            pivotPos.setDefault(position);
        }).until(() -> getError(position) < Constants.AlgaeIntake.MAX_ERROR);


    }

    //Changes the state of the intake
    public Command changeState(IntakeState newState) {

        //Will check to see if intake is up, if it, lower intake, else, raise intake
        Command pivotCommand = newState.intakeExtended ? lowerIntake() : raiseIntake();
        Command intakeCommand = newState.intakeExtended
                //Checks to see if state reverses intake, if it does then reverse intake, if not run intake
                ? (newState.direction == Direction.REVERSED ? reverseIntake() : runIntake())

                //Will stop the intake if not extended
                : stopIntake();
        return pivotCommand.andThen(intakeCommand);
    }
}