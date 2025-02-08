package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {  
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DigitalInput limitSwitch;
  
  private final NetworkTable elevatorTable;
  private final DoublePublisher positionPub;
  private final DoublePublisher currentPub;
  private final BooleanPublisher limitSwitchPub;
  private final BooleanPublisher secondStagePub;
  private final DoubleSubscriber setpointSub;

  
  // State tracking
  private boolean resetElevator;
  private boolean openLoop;
  private boolean inSecondStage;
  
  // Set proper constants later
  private static final double SPIKE_POSITION = 5.0;
  private static final double STAGE_TRANSITION_HEIGHT = 25.0;
  private static final double CURRENT_LIMIT = 35.0;
  private static final double SLOW_ZONE_THRESHOLD = 2.0;
  private final MotionMagicVoltage motionMagic;


  private Elevator() {    
    limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PORT);
    motionMagic = new MotionMagicVoltage(0); //TODO: configure motion magic parameters

    leftMotor = Constants.Elevator.ELEVATOR_LEFT.createTalon();
    rightMotor = Constants.Elevator.ELEVATOR_RIGHT.createTalon();

    //network table variables
    elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
    positionPub = elevatorTable.getDoubleTopic("Position").publish();
    currentPub = elevatorTable.getDoubleTopic("Current").publish();
    limitSwitchPub = elevatorTable.getBooleanTopic("LimitSwitch").publish();
    secondStagePub = elevatorTable.getBooleanTopic("SecondStageActive").publish();
    setpointSub = elevatorTable.getDoubleTopic("Setpoint").subscribe(0.0);

    //setting follower
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), Constants.Elevator.ELEVATOR_LEFT.inversion != Constants.Elevator.ELEVATOR_RIGHT.inversion));
    
    //creating config for software limit switch bec sushi lib doesnt handle
    var config = new TalonFXConfiguration();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.MOTOR_MAX_HEIGHT.in(Constants.CustomUnits.TalonEncoderCounts);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    rightMotor.getConfigurator().apply(config);

    resetElevator = false;
    openLoop = false;
  }

  // moves elevator to defined ElevatorState position *last year specifically checked for elevator idle state
  public Command changeState(ElevatorState state) {
    return runOnce(() -> {
      openLoop = false;
      elevatorTable.getDoubleTopic("Setpoint").publish().set(state.getPos().magnitude());
    }).andThen(new WaitUntilCommand(elevatorInPosition(state.getPos())))
      .andThen(state == ElevatorState.IDLE ? resetElevator() : Commands.none());
  }

  // checks if elevator has reached target position
  private BooleanSupplier elevatorInPosition(Distance targetPos) {
    return () -> Math.abs(rightMotor.getPosition().getValueAsDouble() - targetPos.magnitude()) 
      < Constants.Elevator.MAX_ERROR.in(Units.Rotations); // TODO: need to set max error properly
  }

  // uses limit switch to zero elevator
  public Command resetElevator() {
    return runOnce(() -> {
      rightMotor.set(-0.1);
      resetElevator = true;
    }).andThen(Commands.waitUntil(this::elevatorAtBottom))
      .andThen(runOnce(() -> {
      rightMotor.set(0.0);
      resetElevator = false;
      rightMotor.setPosition(0.0); // zeroes
    }));
  }

  // if pid isnt working
  public Command runOpenLoop(double setSpeed) {
    return runOnce(() -> {
      rightMotor.set(setSpeed);
      openLoop = true;
    });
  }


  public Command stopElevator() {
    return runOnce(() -> {
      rightMotor.set(0.0);
      openLoop = false;
    });
  }

  private boolean elevatorAtBottom() {
    return !limitSwitch.get() || currentSpike();
  }

  private boolean currentSpike() {
    return (rightMotor.getPosition().getValueAsDouble() < SPIKE_POSITION && 
      rightMotor.getSupplyCurrent().getValueAsDouble() > CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    positionPub.set(rightMotor.getPosition().getValueAsDouble());
    currentPub.set(rightMotor.getSupplyCurrent().getValueAsDouble());
    limitSwitchPub.set(elevatorAtBottom());
    secondStagePub.set(inSecondStage);


    // tracking stage and position
    double currentPosition = rightMotor.getPosition().getValueAsDouble();
    inSecondStage = currentPosition > STAGE_TRANSITION_HEIGHT;

    if (elevatorAtBottom()) {
      rightMotor.setPosition(0.0);
    }

    if (!resetElevator && !openLoop) {
      // calculate feedforward with target pos
      double targetPosition = setpointSub.get();
      double feedforward = Constants.Elevator.feedforward.calculate(0.1); //set velocity after testing
      
      if (Math.abs(currentPosition - STAGE_TRANSITION_HEIGHT) < SLOW_ZONE_THRESHOLD) {
          feedforward *= 0.5; // reduce speed when switching stages
      }
      
      rightMotor.setControl(
        motionMagic
            .withPosition(targetPosition)
            .withFeedForward(feedforward)
      );
    }
  }
}