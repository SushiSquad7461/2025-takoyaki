package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ManipulatorState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ElevatorState;


public class StateMachine extends SubsystemBase {
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;
    private final StringPublisher shooterStatePub;
    private final StringPublisher manipulatorStatePub;
    private final StringPublisher elevatorStatePub;

    public enum RobotState {

        IDLE(ShooterState.IDLE, ManipulatorState.IDLE, ElevatorState.IDLE),
        INTAKE_CORAL(ShooterState.IDLE, ManipulatorState.INTAKE, ElevatorState.IDLE),

        // prepare states for different levels
        PREPARE_L1(ShooterState.IDLE, ManipulatorState.IDLE, ElevatorState.L1),
        PREPARE_L2(ShooterState.IDLE, ManipulatorState.IDLE, ElevatorState.L2),
        PREPARE_L3(ShooterState.IDLE, ManipulatorState.IDLE, ElevatorState.L3),
        PREPARE_L4(ShooterState.IDLE, ManipulatorState.IDLE, ElevatorState.L4),

        // scoring states for different levels
        SCORE_L1( ShooterState.IDLE, ManipulatorState.SCORE_L1, ElevatorState.L1),
        SCORE_L2(ShooterState.IDLE, ManipulatorState.SCORE_L2, ElevatorState.L2),
        SCORE_L3(ShooterState.IDLE, ManipulatorState.SCORE_L3, ElevatorState.L3),
        SCORE_L4(ShooterState.IDLE, ManipulatorState.SCORE_L4, ElevatorState.L4),
        
        // special state
        KNOCK_ALGAE(ShooterState.IDLE, ManipulatorState.KNOCK, ElevatorState.L3_KNOCK),
        SHOOT_PROCESSOR(ShooterState.SHOOT_PROCESSOR,ManipulatorState.IDLE,ElevatorState.IDLE),
        SHOOT_BARGE(ShooterState.SHOOT_BARGE,ManipulatorState.IDLE, ElevatorState.IDLE),
        HOLDING_ALGAE(ShooterState.HOLDING,ManipulatorState.IDLE,ElevatorState.IDLE);

     //  public final IntakeState intakeState;
        public final ManipulatorState manipulatorState;
        public final ElevatorState elevatorState;
        public final ShooterState shooterState;
                private RobotState(ShooterState shooterState, ManipulatorState manipulatorState, ElevatorState elevatorState) {
   
                    this.manipulatorState = manipulatorState;
                    this.elevatorState = elevatorState;
                    this.shooterState = shooterState;
        }
    }

    private RobotState state;
   // private final Intake intake;
    private final Shooter shooter;
    private final CoralManipulator manipulator;
    private final Elevator elevator;

    public StateMachine(Shooter shooter, CoralManipulator manipulator, Elevator elevator) {
      //  this.intake = intake;
        this.manipulator = manipulator;
        this.elevator = elevator;
        this.state = RobotState.IDLE;
        this.shooter = shooter;

        //TODO: add publisher for shooter
        stateTable = NetworkTableInstance.getDefault().getTable("StateMachine");
        currentStatePub = stateTable.getStringTopic("CurrentState").publish();
        shooterStatePub = stateTable.getStringTopic("SubsystemStates/Shooter").publish();
        manipulatorStatePub = stateTable.getStringTopic("SubsystemStates/Manipulator").publish();
        elevatorStatePub = stateTable.getStringTopic("SubsystemStates/Elevator").publish();
    }

    @Override
    public void periodic() {
        publishStates();
    }

    public void scheduleNewState(RobotState newState) {
        changeState(newState).schedule();
    }

    public Command changeState(RobotState newState) {
        if (isScoreState(newState)) {
            return Commands.sequence(
                Commands.runOnce(() -> {
                    state = newState;
                    System.out.println(newState.toString() + " scheduled");
                }),
                Commands.parallel(
                    elevator.changeState(newState.elevatorState),

                    /* Makes sure that the shooter is not shooting or
                     * intaking anything while the elevator is about to shoot.
                     * This will prevent any problems with the algae
                     * possibly popping (oops)
                    */
                    (state.shooterState != ShooterState.SHOOT_BARGE && 
                    state.shooterState != ShooterState.SHOOT_BARGE &&
                    state.shooterState != ShooterState.INTAKE) ? 
                       shooter.changeState(newState.shooterState) : 
                       Commands.none() 

                   ), Commands.sequence(
                    manipulator.changeState(newState.manipulatorState)
                )
            );   
        } 
        return Commands.sequence(
            Commands.runOnce(() -> {
                state = newState;
                System.out.println(newState.toString() + " scheduled");
            }),
            Commands.parallel(
                //If the elevator is not shooting, then the shooter will just change states normally
                elevator.changeState(newState.elevatorState),
                manipulator.changeState(newState.manipulatorState),
                shooter.changeState(newState.shooterState)
            )
        );

    }

    private boolean isScoreState(RobotState state) {
        return state == RobotState.SCORE_L1 || 
               state == RobotState.SCORE_L2 || 
               state == RobotState.SCORE_L3 || 
               state == RobotState.SCORE_L4;
    }

    public RobotState getCurrentState() {
        return state;
    }
    //will just publish the states of each subsystem 
    private void publishStates() {
        currentStatePub.set(state.toString());
        elevatorStatePub.set(state.elevatorState.toString());
        shooterStatePub.set(state.shooterState.toString());
        manipulatorStatePub.set(state.manipulatorState.toString());
    }
}