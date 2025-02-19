package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DigitalInput limitSwitch;

  private final NetworkTable elevatorTable;
  private final DoublePublisher positionPub;
  private final DoublePublisher currentPub;
  private final BooleanPublisher limitSwitchPub;
  public SysIdRoutine routine;

  private static final Current CURRENT_LIMIT = Amps.of(35);

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  public Elevator() {
    limitSwitch = new DigitalInput(Constants.Ports.LIMIT_SWITCH_PORT);

    leftMotor = Constants.Elevator.ELEVATOR_LEFT.createTalon();
    rightMotor = Constants.Elevator.ELEVATOR_RIGHT.createTalon();

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
          leftMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
          rightMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
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
    return runOnce(() -> {
      elevatorTable.getDoubleTopic("Setpoint").publish().set(state.getPos().in(Inches));
    }).andThen(new WaitUntilCommand(elevatorInPosition(state.getPos())))
        .andThen(state == ElevatorState.IDLE || state == ElevatorState.L1 ? resetElevator() : Commands.none());
  }

  // checks if elevator has reached target position
  private BooleanSupplier elevatorInPosition(Distance targetPos) {
    Angle targetAngle = (Angle) targetPos.div(Constants.Elevator.ELEVATOR_EXTENSION_PER_MOTOR_ANGLE).div(Constants.Elevator.GEAR_RATIO); //converts target position into angle
    return () -> Math.abs(rightMotor.getPosition().getValue().in(Rotations)
        - targetAngle.in(Rotations)) < Constants.Elevator.MAX_ERROR.in(Rotations); //TODO: need to set max error properly
  }

  // uses limit switch to zero elevator
  public Command resetElevator() {
    return runOnce(() -> {
      rightMotor.set(-0.1);
    }).andThen(Commands.waitUntil(this::elevatorAtBottom))
      .andThen(runOnce(() -> {
        rightMotor.set(0.0);
        rightMotor.setPosition(0.0); //zeroes
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
    return limitSwitch.get() || currentSpike();
  }

  private boolean currentSpike() {
    return (rightMotor.getSupplyCurrent().getValue().compareTo(CURRENT_LIMIT) > 0);
  }

  @Override
  public void periodic() {
    final var pos = rightMotor.getPosition();
    final var current = rightMotor.getSupplyCurrent();
    
    if (!pos.getStatus().isError() && !current.getStatus().isError()) {
      positionPub.set(pos.getValueAsDouble());
      currentPub.set(current.getValueAsDouble());
    }

    limitSwitchPub.set(elevatorAtBottom());
  }

}
