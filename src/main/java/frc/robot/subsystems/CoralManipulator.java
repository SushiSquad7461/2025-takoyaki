package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.util.control.nt.PIDTuning;
import frc.util.control.nt.TunableNumber;

import java.util.function.BooleanSupplier;

public class CoralManipulator extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final DigitalInput beambreak;
    private final MotionMagicVoltage motionMagic;
    
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
    private final TunableNumber kG;
    private final PIDTuning pidTuning;

    private final NetworkTable wristTable;
    private final DoublePublisher anglePub;
    private final BooleanPublisher beambreakPub;
    private final DoublePublisher currentPub;
    
    private boolean openLoop;
    private Measure<AngleUnit> targetAngle;

    private static final Measure<CurrentUnit> CURRENT_LIMIT = Amps.of(35);
    private static final Measure<VoltageUnit> VOLTAGE_LIMIT = Volts.of(12);

    public CoralManipulator() {
        pivotMotor = new TalonFX(Constants.Pivot.PIVOT_MOTOR_ID);
        rollerMotor = new TalonFX(Constants.Pivot.ROLLER_MOTOR_ID);
        beambreak = new DigitalInput(Constants.Pivot.BEAM_BREAK_PORT);
  
        motionMagic = new MotionMagicVoltage(0);
        
        kP = new TunableNumber("Wrist/PID/kP", Constants.Pivot.kP, Constants.TUNING_MODE);
        kI = new TunableNumber("Wrist/PID/kI", Constants.Pivot.kI, Constants.TUNING_MODE);
        kD = new TunableNumber("Wrist/PID/kD", Constants.Pivot.kD, Constants.TUNING_MODE);
        kG = new TunableNumber("Wrist/PID/kG", Constants.Pivot.kG, Constants.TUNING_MODE);
        pidTuning = Constants.Pivot.PIVOT_CONFIG.genPIDTuning("Manipulator Pivot", Constants.TUNING_MODE);

        // configure pivot motor
        var config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.Pivot.kP;
        config.Slot0.kI = Constants.Pivot.kI;
        config.Slot0.kD = Constants.Pivot.kD;
        
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Pivot.MAX_ANGLE.in(Radians);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Pivot.MIN_ANGLE.in(Radians);
        
        // current and voltage limits
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = VOLTAGE_LIMIT.in(Volts);
        config.Voltage.PeakReverseVoltage = -VOLTAGE_LIMIT.in(Volts);
        
        pivotMotor.getConfigurator().apply(config);
        
        // network table setup
        wristTable = NetworkTableInstance.getDefault().getTable("Wrist");
        anglePub = wristTable.getDoubleTopic("Angle").publish();
        beambreakPub = wristTable.getBooleanTopic("BeamBreak").publish();
        currentPub = wristTable.getDoubleTopic("Current").publish();
        
        openLoop = false;
        targetAngle = Degrees.of(0);
    }

    public Command changeState(ManipulatorState state) {
        return runOnce(() -> {
            openLoop = false;
            targetAngle = state.getAngle();
        })
        .andThen(new WaitUntilCommand(atTargetPosition()))
        .andThen(state == ManipulatorState.IDLE ? stopRollers() : Commands.none());
    }
    
    public Command runRollers(double speed) {
        return runOnce(() -> rollerMotor.set(speed));
    }
    
    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }
    
    public Command setOpenLoop(double speed) {
        return runOnce(() -> {
            openLoop = true;
            pivotMotor.set(speed);
        });
    }
    
    private BooleanSupplier atTargetPosition() {
        return () -> Math.abs(getPosition().in(Radians) - targetAngle.in(Radians)) 
            < Constants.Pivot.ANGLE_TOLERANCE.in(Radians);
    }
    
    public Measure<AngleUnit> getPosition() {
        return Radians.of(pivotMotor.getPosition().getValueAsDouble());
    }
    
    public boolean hasCoral() {
        return !beambreak.get(); //TODO: check how beambreak works (get() returns true when circuit is closed afaik)
    }

    @Override
    public void periodic() {
        anglePub.set(getPosition().in(Degrees));
        beambreakPub.set(hasCoral());
        currentPub.set(pivotMotor.getSupplyCurrent().getValueAsDouble());
        
        if (!openLoop) {
            pidTuning.updatePID(pivotMotor);
            
            // double ff = Constants.Pivot.PIVOT_FEEDFORWARD.calculate(
            //     getPosition().in(Radians), 
            //     0  
            // );
            
            //TODO: sam i wont need to apply feedforward here if alr set in constants right, even if its arm feedforward?
            pivotMotor.setControl(
                motionMagic
                    .withPosition(targetAngle.in(Radians))
                    // .withFeedForward(ff)
            );
        }
    }
}