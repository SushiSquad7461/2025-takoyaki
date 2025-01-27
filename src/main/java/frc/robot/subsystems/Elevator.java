package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;

public class Elevator extends SubsystemBase {
  private static Elevator instance;
  
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DigitalInput limitSwitch;
  
  private PIDTuning pid;
  private final TunableNumber setpoint;
  private final ElevatorFeedforward ffd; // Down direction feedforward
  private final ElevatorFeedforward ffu; // Up direction feedforward
  
  // State tracking
  private boolean up;
  private boolean resetElevator;
  private boolean openLoop;
  private boolean inSecondStage;
  
  // Set proper constants later
  private static final double STAGE_TRANSITION_HEIGHT = 25.0;
  private static final double STAGE_1_CURRENT_LIMIT = 30.0;
  private static final double STAGE_2_CURRENT_LIMIT = 35.0;
  private static final double SLOW_ZONE_THRESHOLD = 2.0;
  
  public static Elevator getInstance() {
    if (instance == null)
      instance = new Elevator();
    return instance;
  }

  private Elevator() {
    ffd = new ElevatorFeedforward(0.0, Constants.Elevator.G_DOWN, 0.0);
    ffu = new ElevatorFeedforward(0.0, Constants.Elevator.G_UP, 0.0);
    
    limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PORT);
    leftMotor = Constants.Elevator.ELEVATOR_LEFT.createTalon();
    rightMotor = Constants.Elevator.ELEVATOR_RIGHT.createTalon();
    
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
    
    //creating config for software limit switch bec sushi lib doesnt handle
    var config = new TalonFXConfiguration();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.MAX_HEIGHT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    rightMotor.getConfigurator().apply(config);
    
    resetElevator = false;
    openLoop = false;
    inSecondStage = false;
    up = true;
    
    if (Constants.TUNING_MODE) {
      pid = new PIDTuning("Elevator PID", Constants.Elevator.ELEVATOR_RIGHT.pid, Constants.TUNING_MODE);
    }
    
    setpoint = new TunableNumber("Elevator Setpoint", 0, Constants.TUNING_MODE);
  }

  // moves elevator to defined ElevatorState position *last year specifically checked for elevator idle state
  public Command changeState(ElevatorState state) {
    return runOnce(() -> {
      openLoop = false;
      up = state.getPos() > rightMotor.getPosition().getValueAsDouble();
      setpoint.setDefault(state.getPos());
    }).andThen(new WaitUntilCommand(elevatorInPosition(state.getPos())))
      .andThen(state == ElevatorState.IDLE ? resetElevator() : Commands.none());
  }

  // checks if elevator has reached target position
  private BooleanSupplier elevatorInPosition(double targetPos) {
    return () -> Math.abs(rightMotor.getPosition().getValueAsDouble() - targetPos) 
      < Constants.Elevator.MAX_ERROR; // TODO: need to set max error properly
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
  public Command runOpenLoopUp() {
    return runOnce(() -> {
      double speed = inSecondStage ? 0.15 : 0.2; // slows down second stage
      rightMotor.set(speed);
      openLoop = true;
    });
  }

  public Command runOpenLoopDown() {
    return runOnce(() -> {
      double speed = inSecondStage ? -0.15 : -0.2;
      rightMotor.set(speed);
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
    double currentLimit = inSecondStage ? STAGE_2_CURRENT_LIMIT : STAGE_1_CURRENT_LIMIT;
    return (rightMotor.getPosition().getValueAsDouble() < 5 && 
      rightMotor.getSupplyCurrent().getValueAsDouble() > currentLimit 
      && !up);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", rightMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Current", rightMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator Limit Switch", elevatorAtBottom());
    SmartDashboard.putBoolean("Second Stage Active", inSecondStage);

    if (Constants.TUNING_MODE) {
      pid.updatePID(rightMotor);
    }

    // tracking stage and position
    double currentPosition = rightMotor.getPosition().getValueAsDouble();
    inSecondStage = currentPosition > STAGE_TRANSITION_HEIGHT;

    if (elevatorAtBottom()) {
      rightMotor.setPosition(0.0);
    }

    if (!resetElevator && !openLoop) {
      // calculate feedforward with target pos
      double targetPosition = setpoint.get();
      double feedforward = up ? ffu.calculate(0.1) : ffd.calculate(0.1); //set velocity after testing
      
      if (Math.abs(currentPosition - STAGE_TRANSITION_HEIGHT) < SLOW_ZONE_THRESHOLD) {
          feedforward *= 0.5; // reduce speed when switching stages
      }
      
      rightMotor.setControl(new PositionDutyCycle(targetPosition).withFeedForward(feedforward));
    }
  }
}