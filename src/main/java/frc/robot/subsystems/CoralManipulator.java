package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final DigitalInput beambreak;
    private final MotionMagicVoltage motionMagic;
    public SysIdRoutine routine;
    
    private final PIDTuning pidTuning;
    private final NetworkTable manipulatorTable;
    private final BooleanPublisher beambreakPub;
    private final DoublePublisher currentPub;
    private boolean openLoop;
    private boolean changedPos;
    private Angle targetAngle;

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    public CoralManipulator() {
        rollerMotor = Constants.CoralManipulator.ROLLER_CONFIG.createTalon();
        beambreak = new DigitalInput(Constants.Ports.BEAM_BREAK_PORT);
        
        // network table setup
        manipulatorTable = NetworkTableInstance.getDefault().getTable("Manipulator");
        beambreakPub = manipulatorTable.getBooleanTopic("BeamBreak").publish();
        currentPub = manipulatorTable.getDoubleTopic("Current").publish();
        openLoop = false;
        changedPos = false;
        targetAngle = Degrees.of(0);

        // Creates a SysIdRoutine
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
              rollerMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
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
      if (state == ManipulatorState.SCORE && !hasCoral()) {
          return Commands.none();
      }

      // else, default to running rollers
      return runRollers(state.getRollerSpeed());
  }

  public Command runRollers(double speed) {
      return runOnce(() -> rollerMotor.set(speed));
  }

  public Command stopRollers() {
      return runOnce(() -> rollerMotor.set(0));
  }

  public boolean hasCoral() {
      return !beambreak.get(); //get() returns true when circuit is closed
  }

  @Override
  public void periodic() {
      beambreakPub.set(hasCoral());
      currentPub.set(rollerMotor.getSupplyCurrent().getValueAsDouble());
  }
  
}
