package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

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
  private final DoublePublisher setpointPub;
  public SysIdRoutine routine;
  private boolean reset = false;

  private static final double CURRENT_LIMIT_AMPS = Robot.isReal() ? 5 : 4;

  private final VoltageOut voltReq = new VoltageOut(0);

  private double simCurrentDrawAmps = 0;
  private final TalonFXSimState leftSim;
  private final TalonFXSimState rightSim;
  private final double maxSimHeightMeters = ElevatorState.L4.position.plus(Inches.of(2)).in(Meters);
  private final ElevatorSim sim = new ElevatorSim(
      DCMotor.getKrakenX60(2), 
      Constants.Elevator.GEAR_RATIO,
      10, // TODO fix/verify this
      Constants.Elevator.ELEVATOR_EXTENSION_PER_ROTATION.div(2.*Math.PI).in(Meters),
      0.,
      maxSimHeightMeters,
      true,
      0.);
  private final Mechanism2d mech2d = new Mechanism2d(1, maxSimHeightMeters);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 0, 0);
  private final MechanismLigament2d elevMech2d = mech2dRoot.append(new MechanismLigament2d(
    "Elevator", 
    sim.getPositionMeters(), 
    90));

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

    if(Robot.isSimulation()) SmartDashboard.putData("Elevator2d", mech2d);
    SmartDashboard.putData("Elev L4", changeState(ElevatorState.L4));
    SmartDashboard.putData("Elev L3", changeState(ElevatorState.L3));
    SmartDashboard.putData("Elev L2", changeState(ElevatorState.L2));
    SmartDashboard.putData("Elev L1", changeState(ElevatorState.L1));
    SmartDashboard.putData("Elev Idle", changeState(ElevatorState.IDLE));
    SmartDashboard.putData("Elev Stop", stopElevator());
    leftSim = leftMotor.getSimState();
    rightSim = rightMotor.getSimState();
    leftSim.Orientation = ChassisReference.Clockwise_Positive;
    rightSim.Orientation = ChassisReference.CounterClockwise_Positive;

    // network table variables
    elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
    limitSwitchPub = elevatorTable.getBooleanTopic("LimitSwitch").publish();
    setpointPub = elevatorTable.getDoubleTopic("Setpoint").publish();

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
    SmartDashboard.putData("ElevSysIdQuasiFwd", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("ElevSysIdQuasiRev", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("ElevSysIdDynFwd", sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("ElevSysIdDynRev", sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
      setpointPub.set(state.position.in(Inches));
      rightMotor.setControl(motionMagic.withPosition(state.targetMotorRotations));
      if(state != ElevatorState.IDLE) reset = false;
    })
    .andThen(Commands.waitUntil(() -> elevatorInPosition(state)))
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

  private static double heightMetersToMotorRotations(double heightMeters) {
    return heightMeters 
      * Constants.Elevator.GEAR_RATIO
      / Constants.Elevator.ELEVATOR_EXTENSION_PER_ROTATION.in(Meters);
  }

  @Override
  public void simulationPeriodic() {
    var supplyVoltage = RobotController.getBatteryVoltage();
    leftSim.setSupplyVoltage(supplyVoltage);
    rightSim.setSupplyVoltage(supplyVoltage);
    sim.setInputVoltage(rightSim.getMotorVoltage());
    sim.update(Constants.LOOP_TIME_SECONDS);
    
    var elevExtension = sim.getPositionMeters();
    var motorRot = heightMetersToMotorRotations(elevExtension);
    leftSim.setRawRotorPosition(motorRot);
    rightSim.setRawRotorPosition(motorRot);
    var motorRotPerSec = heightMetersToMotorRotations(sim.getVelocityMetersPerSecond());
    leftSim.setRotorVelocity(motorRotPerSec);
    rightSim.setRotorVelocity(motorRotPerSec);
    
    elevMech2d.setLength(elevExtension);
    simCurrentDrawAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  public double getSimulatedCurrentDrawAmps() {
    return simCurrentDrawAmps;
  }
}
