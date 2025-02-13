package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.util.control.nt.PIDTuning;

import java.util.function.BooleanSupplier;

public class CoralManipulator extends SubsystemBase {
    // private final TalonFX pivotMotor;
    // private final DoublePublisher anglePub;
    // private final MotionMagicVoltage motionMagic;
    // private final PIDTuning pidTuning;
    // private boolean openLoop;
    // private boolean changedPos;
    // private Angle targetAngle;

    private final TalonFX rollerMotor;
    private final DigitalInput beambreak;
    private final NetworkTable manipulatorTable;
    private final BooleanPublisher beambreakPub;
    private final DoublePublisher currentPub;

    public CoralManipulator() {
        rollerMotor = Constants.CoralManipulator.ROLLER_CONFIG.createTalon();
        beambreak = new DigitalInput(Constants.Ports.BEAM_BREAK_PORT);
        // pivotMotor = Constants.CoralManipulator.PIVOT_CONFIG.createTalon();
        // motionMagic = new MotionMagicVoltage(0);
        // pidTuning = Constants.CoralManipulator.PIVOT_CONFIG.genPIDTuning("Manipulator Pivot", Constants.TUNING_MODE);

        
        // network table setup
        manipulatorTable = NetworkTableInstance.getDefault().getTable("Manipulator");
        beambreakPub = manipulatorTable.getBooleanTopic("BeamBreak").publish();
        currentPub = manipulatorTable.getDoubleTopic("Current").publish();
        // anglePub = manipulatorTable.getDoubleTopic("Angle").publish();

        // openLoop = false;
        // changedPos = false;
        // targetAngle = Degrees.of(0);
    }

    // public Command changeState(ManipulatorState state) {
    //     return runOnce(() -> {
    //         openLoop = false;
    //         targetAngle = state.getAngle();
    //         changedPos = true;
    //     })
    //     .andThen(new WaitUntilCommand(atTargetPosition()))
    //     .andThen(state == ManipulatorState.IDLE ? stopRollers() : runRollers(state.getRollerSpeed()));
    // }

    public Command changeState(ManipulatorState state) {
        if (state == ManipulatorState.IDLE) {
            return stopRollers();
        }
        
        if (state == ManipulatorState.INTAKE) {
            // for intake, run until detect coral
            return runRollers(state.getRollerSpeed())
                .until(this::hasCoral);
        }

        // for scoring, only run if coral piece is detected
        if (isScoreState(state) && !hasCoral()) {
            return Commands.none();
        }

        // else, default to running rollers
        return runRollers(state.getRollerSpeed());
    }
    
    private boolean isScoreState(ManipulatorState state) {
        return state == ManipulatorState.L1 || 
               state == ManipulatorState.L2 || 
               state == ManipulatorState.L3 || 
               state == ManipulatorState.L4;
    }
    
    public Command runRollers(double speed) {
        return runOnce(() -> rollerMotor.set(speed));
    }
    
    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }
    
    // public Command setOpenLoop(double speed) {
    //     return runOnce(() -> {
    //         openLoop = true;
    //         pivotMotor.set(speed);
    //     });
    // }
    
    // private BooleanSupplier atTargetPosition() {
    //     return () -> Math.abs(getPosition().in(Radians) - targetAngle.in(Radians)) 
    //         < Constants.CoralManipulator.ANGLE_TOLERANCE.in(Radians);
    // }
    
    // public Angle getPosition() {
    //     return Radians.of(pivotMotor.getPosition().getValueAsDouble());
    // }
    
    public boolean hasCoral() {
        return !beambreak.get(); //get() returns true when circuit is closed
    }

    // @Override
    // public void periodic() {
    //     anglePub.set(getPosition().in(Degrees));
    //     beambreakPub.set(hasCoral());
    //     currentPub.set(pivotMotor.getSupplyCurrent().getValueAsDouble());
        
    //     if (!openLoop) {
    //         pidTuning.updatePID(pivotMotor);
            
    //         if (changedPos) {
    //             pivotMotor.setControl(
    //                 motionMagic
    //                     .withPosition(targetAngle.in(Radians))
    //             );
    //             changedPos = false;
    //         }
    //     }
    // }

    @Override
    public void periodic() {
        beambreakPub.set(hasCoral());
        currentPub.set(rollerMotor.getSupplyCurrent().getValueAsDouble());
    }
}