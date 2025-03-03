package frc.robot.subsystems;

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
  private final BooleanPublisher limitSwitchPub;
  public SysIdRoutine routine;
  private boolean reset = false;

  private static final double CURRENT_LIMIT_AMPS = 5;

  private final VoltageOut voltReq = new VoltageOut(0);

  public Elevator() {
    limitSwitch = new DigitalInput(Constants.Ports.LIMIT_SWITCH_PORT);
    leftMotor = new TalonFX(Constants.Ports.ELEVATOR_LEFT_ID);
    leftMotor.getConfigurator().apply(Constants.Elevator.ELEVATOR_LEFT);
    rightMotor = new TalonFX(Constants.Ports.ELEVATOR_RIGHT_ID);
    rightMotor.getConfigurator().apply(Constants.Elevator.ELEVATOR_RIGHT);
    rightMotorCurrent = rightMotor.getSupplyCurrent();
    rightMotorPosition = rightMotor.getPosition();
    statusSignals = new StatusSignal[]{rightMotorCurrent, rightMotorPosition};
    motionMagic = new MotionMagicVoltage(0);

    // network table variables
    elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
    limitSwitchPub = elevatorTable.getBooleanTopic("LimitSwitch").publish();

    // setting follower
    leftMotor.setControl(new Follower(
      rightMotor.getDeviceID(), 
      Constants.Elevator.ELEVATOR_LEFT.MotorOutput.Inverted != Constants.Elevator.ELEVATOR_RIGHT.MotorOutput.Inverted
    ));

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
      // elevatorTable.getDoubleTopic("Setpoint").publish().set(state.position.in(Inches));
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
      System.out.print("resetElevator triggered");
      rightMotor.set(-0.1);
    }).andThen(Commands.waitUntil(this::elevatorAtBottom))
      .andThen(runOnce(() -> {
        rightMotor.set(0.0);
        rightMotor.setPosition(0.0); //zeroes
        reset = true;
        System.out.print("elevator reached bottom ");
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

  public Command goUp() {
    return run(() -> rightMotor.set(.1)).finallyDo(() -> rightMotor.set(0));
  }


  public Command goDown() {
    return run(() -> rightMotor.set(-.1)).finallyDo(() -> rightMotor.set(0));
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(statusSignals);
    limitSwitchPub.set(limitSwitch.get());
  }
}
