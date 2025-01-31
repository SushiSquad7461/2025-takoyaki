package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;


import frc.util.Motor.MotorHelper;
import frc.util.Sensors.absoluteEncoder.AbsoluteEncoder;
import frc.util.SmartDashboard.PIDTuning;
import frc.util.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

public class BetaIntake extends Intake {
    private final SparkMax pivotMotor;

    private static BetaIntake instance;

    private final ArmFeedforward intakeFeedforward;
    private final AbsoluteEncoder absoluteEncoder;

    private TunableNumber pivotPos;

    private PIDTuning pivotPID;

    public static BetaIntake getInstance() {
        if (instance == null) {
            return new BetaIntake();
        }
        return instance;
    }

    private BetaIntake() {
        super();

        pivotMotor = Constants.Intake.PIVOT_CONFIG.createSparkMax();

        intakeFeedforward = new ArmFeedforward(0.0, Constants.Intake.G, 0.0);
        absoluteEncoder = new AbsoluteEncoder(Constants.Intake.ENCODER_CHANNEL, Constants.Intake.ENCODER_ANGLE_OFFSET,
                false);
        MotorHelper.setDegreeConversionFactor(pivotMotor, Constants.Intake.INTAKE_GEAR_RATIO);

        resetToAbsolutePosition();

        pivotPos = new TunableNumber("Intake Pos", Constants.Intake.RAISED_POS, Constants.TUNING_MODE);
        pivotPID = Constants.Intake.PIVOT_CONFIG.genPIDTuning("Pivot Intake", Constants.TUNING_MODE);
    }

    public void resetToAbsolutePosition() {
        pivotMotor.getEncoder().setPosition(getAbsolutePosition());
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getNormalizedPosition();
    }

    public double getPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    //Returns if error is less then max error
    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < Constants.Intake.MAX_ERROR);
    }

    public double getAbsoluteError() {
        return Math.abs(getPosition() - getAbsolutePosition());
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

    public Command changePivotPos(double position) {
        return run(() -> {
            //Will set pivot position as default position until error is less then max error
            pivotPos.setDefault(position);
        }).until(() -> getError(position) < Constants.Intake.MAX_ERROR);
    }

    @Override
    public void periodic() {

        if (getAbsoluteError() > Constants.Intake.ERROR_LIMIT) {
            resetToAbsolutePosition();
        }

        SmartDashboard.putNumber("Intake Absolute Encoder", getAbsolutePosition());
        SmartDashboard.putNumber("Intake Relative Encoder", pivotMotor.getEncoder().getPosition());

        pivotPID.updatePID(pivotMotor);

        pivotMotor.getClosedLoopController().setReference(

                pivotPos.get(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                intakeFeedforward.calculate(Math.toRadians(getPosition()), 0.0));

        super.periodic();
    }
}