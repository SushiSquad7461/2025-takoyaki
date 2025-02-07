package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;


import frc.util.Motor.MotorHelper;
import frc.util.SmartDashboard.PIDTuning;
import frc.util.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;


public class BetaIntake extends Intake {
    private final TalonFX pivotMotor;
    private final TalonFX wheelMotor;

   // private final ArmFeedforward intakeFeedforward;

    private TunableNumber pivotPos;

    private PIDTuning pivotPID;


    private BetaIntake() {
        super();

        pivotMotor = new TalonFX( 1);
        wheelMotor = new TalonFX( 2 );

        MotorHelper.setDegreeConversionFactor(pivotMotor, Constants.Intake.INTAKE_GEAR_RATIO);
        pivotPos = new TunableNumber("Intake Pos", Constants.Intake.RAISED_POS, Constants.TUNING_MODE);
        pivotPID = Constants.Intake.PIVOT_CONFIG.genPIDTuning("Pivot Intake", Constants.TUNING_MODE);
    }





    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    //Returns if error is less then max error
    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < Constants.Intake.MAX_ERROR);
    }


    
    public double getError(double setpoint) {
        return Math.abs(getPosition() - setpoint);
    }

    @Override
    public Command lowerIntake() {
        return changePivotPos(Constants.Intake.LOWERED_POS);
    }

    @Override
    public Command raiseIntake() {
        return changePivotPos(Constants.Intake.RAISED_POS);
    }

    private Command changePivotPos(double position) {
        return run(() -> {
            //Will set pivot position as default position until error is less then max error
            pivotPos.setDefault(position);
        }).until(() -> getError(position) < Constants.Intake.MAX_ERROR);
    }
}