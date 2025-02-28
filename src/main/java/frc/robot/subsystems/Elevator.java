package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.control.nt.PIDTuning;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  /** getValueAsDouble returns in Amps */
  private final StatusSignal<Current> rightMotorCurrent;
  /** getValueAsDouble returns in Rotations */
  private final StatusSignal<Angle> rightMotorPosition;
  private final BaseStatusSignal[] statusSignals;
  private final DigitalInput limitSwitch;
  private final MotionMagicVoltage motionMagic;

  private final NetworkTable elevatorTable;
  private final DoublePublisher positionPub;
  private final DoublePublisher currentPub;
  private final BooleanPublisher limitSwitchPub;
  private PIDTuning elevatorRightPID;
  public SysIdRoutine routine;
  private boolean reset = false;

  private static final double CURRENT_LIMIT_AMPS = 5;

  private final VoltageOut voltReq = new VoltageOut(0);

  public Elevator() {
    limitSwitch = new DigitalInput(Constants.Ports.LIMIT_SWITCH_PORT);
    elevatorRightPID = Constants.Elevator.ELEVATOR_LEFT.genPIDTuning("Elevator", Constants.TUNING_MODE);
    leftMotor = Constants.Elevator.ELEVATOR_LEFT.createTalon();
    rightMotor = Constants.Elevator.ELEVATOR_RIGHT.createTalonForMotionMagic();
    rightMotorCurrent = rightMotor.getSupplyCurrent();
    rightMotorPosition = rightMotor.getPosition();
    statusSignals = new StatusSignal[]{rightMotorCurrent, rightMotorPosition};
    motionMagic = new MotionMagicVoltage(0);

    // network table variables
    elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
    positionPub = elevatorTable.getDoubleTopic("Position").publish();
    currentPub = elevatorTable.getDoubleTopic("Current").publish();
    limitSwitchPub = elevatorTable.getBooleanTopic("LimitSwitch").publish();

    // setting follower
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(),
        Constants.Elevator.ELEVATOR_LEFT.inversion != Constants.Elevator.ELEVATOR_RIGHT.inversion));

    routine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null, // Use default ramp rate (1 V/s)
        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
        null, // Use default timeout (10 s)
        // Log state with Phoenix SignalLogger class
        (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        (volts) -> {
          leftMotor.setControl(voltReq.withOutput(volts.in(Volts)));
          rightMotor.setControl(voltReq.withOutput(volts.in(Volts)));
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

  // moves elevator to defined ElevatorState position *last year specifically
  // checked for elevator idle state
  public Command changeState(ElevatorState state) {
    return run(() -> {
      elevatorTable.getDoubleTopic("Setpoint").publish().set(state.position.in(Inches));
      rightMotor.setControl(motionMagic.withPosition(state.targetMotorRotations));
      if(state != ElevatorState.IDLE) reset = false;
    }).until(() -> elevatorInPosition(state))
        .andThen(state == ElevatorState.IDLE && !reset ? resetElevator() : Commands.none());
  }

  // checks if elevator has reached target position
  private boolean elevatorInPosition(ElevatorState state) {
    var maxErr = state == ElevatorState.IDLE && currentSpike() ? Constants.Elevator.RELAXED_MAX_ERROR_ROTATIONS : Constants.Elevator.MAX_ERROR_ROTATIONS;    
    return Math.abs(rightMotorPosition.getValueAsDouble() - state.targetMotorRotations) < maxErr;
  }

  public static Angle heightToMotor(Distance distance) {
    return distance
      .times(Constants.Elevator.GEAR_RATIO)
      .div(Constants.Elevator.ELEVATOR_EXTENSION_PER_ROTATION)
      .times(Rotations.of(1));
  }

  // uses limit switch to zero elevator
  public Command resetElevator() {
    return runOnce(() -> {
      rightMotor.set(-0.1);
    }).andThen(Commands.waitUntil(this::elevatorAtBottom))
      .andThen(runOnce(() -> {
        rightMotor.set(0.0);
        rightMotor.setPosition(0.0); //zeroes
        reset = true;
      })
    );
  }

  // if pid isnt working
  public Command runOpenLoop(double setSpeed) {
    return runOnce(() -> {
      rightMotor.set(setSpeed);
    });
  }

  public Command stopElevator() {
    return runOnce(() -> {
      rightMotor.set(0.0);
    });
  }

  private boolean elevatorAtBottom() {
    return currentSpike(); // || limitSwitch.get()
  }

  private boolean currentSpike() {
    return rightMotorCurrent.getValueAsDouble() > CURRENT_LIMIT_AMPS;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(statusSignals);
    final var pos = rightMotor.getPosition();
    final var current = rightMotor.getSupplyCurrent();
    
    elevatorRightPID.updatePID(rightMotor);

    if (!pos.getStatus().isError() && !current.getStatus().isError()) {
      positionPub.set(pos.getValue().in(Rotations));
      currentPub.set(current.getValue().in(Amp));
    }

    limitSwitchPub.set(limitSwitch.get());
  }

  public Command goUp() {
    return run(() -> rightMotor.set(.1)).finallyDo(() -> rightMotor.set(0));
  }


public Command goDown() {
  return run(() -> rightMotor.set(-.2)).finallyDo(() -> rightMotor.set(0));
}

}
