package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.util.control.nt.PIDTuning;

import java.util.function.BooleanSupplier;



public class CoralManipulator extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final DigitalInput beambreak;
    private final MotionMagicVoltage motionMagic;
    public SysIdRoutine routine;
    
    private final PIDTuning pidTuning;

    private final NetworkTable manipulatorTable;
    private final DoublePublisher anglePub;
    private final BooleanPublisher beambreakPub;
    private final DoublePublisher currentPub;
    
    private boolean openLoop;
    private boolean changedPos;
    private Angle targetAngle;


    public CoralManipulator() {
        pivotMotor = Constants.CoralManipulator.PIVOT_CONFIG.createTalon();
        rollerMotor = Constants.CoralManipulator.ROLLER_CONFIG.createTalon();
        beambreak = new DigitalInput(Constants.Ports.BEAM_BREAK_PORT);
        motionMagic = new MotionMagicVoltage(0);
        pidTuning = Constants.CoralManipulator.PIVOT_CONFIG.genPIDTuning("Manipulator Pivot", Constants.TUNING_MODE);

        
        // network table setup
        manipulatorTable = NetworkTableInstance.getDefault().getTable("Manipulator");
        anglePub = manipulatorTable.getDoubleTopic("Angle").publish();
        beambreakPub = manipulatorTable.getBooleanTopic("BeamBreak").publish();
        currentPub = manipulatorTable.getDoubleTopic("Current").publish();
        
        openLoop = false;
        changedPos = false;
        targetAngle = Degrees.of(0);

        // Creates a SysIdRoutine
    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Voltage voltage) -> { 
  
              double leftPosition = pivotMotor.getPosition().getValueAsDouble();  // Correctly retrieve the position value as double
              double rightPosition = rollerMotor.getPosition().getValueAsDouble();  // Same for right motor            
          
              double leftCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble(); // Correctly retrieve the current value as double
              double rightCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble(); // Same for right motor
  
            },
          
  
          (SysIdRoutineLog log) -> {
              // Log data using SignalLogger
              SignalLogger.writeString("Left Motor Position", "" + pivotMotor.getPosition().getValueAsDouble());
              SignalLogger.writeString("Right Motor Position", "" + rollerMotor.getPosition().getValueAsDouble());
              SignalLogger.writeString("Left Motor Current", "" + pivotMotor.getSupplyCurrent().getValueAsDouble());
              SignalLogger.writeString("Right Motor Current", "" + rollerMotor.getSupplyCurrent().getValueAsDouble());
      
          },
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


    public Command changeState(ManipulatorState state) {
        return runOnce(() -> {
            openLoop = false;
            targetAngle = state.getAngle();
            changedPos = true;
        })
        .andThen(new WaitUntilCommand(atTargetPosition()))
        .andThen(state == ManipulatorState.IDLE ? stopRollers() : runRollers(state.getRollerSpeed()));
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
            < Constants.CoralManipulator.ANGLE_TOLERANCE.in(Radians);
    }
    
    public Angle getPosition() {
        return Radians.of(pivotMotor.getPosition().getValueAsDouble());
    }
    
    public boolean hasCoral() {
        return !beambreak.get(); //get() returns true when circuit is closed
    }

    @Override
    public void periodic() {
        anglePub.set(getPosition().in(Degrees));
        beambreakPub.set(hasCoral());
        currentPub.set(pivotMotor.getSupplyCurrent().getValueAsDouble());
        
        if (!openLoop) {
            pidTuning.updatePID(pivotMotor);
            
            if (changedPos) {
                pivotMotor.setControl(
                    motionMagic
                        .withPosition(targetAngle.in(Radians))
                );
                changedPos = false;
            }
        }
    }
}
