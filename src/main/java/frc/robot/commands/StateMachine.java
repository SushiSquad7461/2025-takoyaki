package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ManipulatorState;
import frc.robot.subsystems.ElevatorState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeState;

public class StateMachine extends SubsystemBase {
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;
    private final StringPublisher intakeStatePub;
    private final StringPublisher manipulatorStatePub;
    private final StringPublisher elevatorStatePub;
    private RobotState targetScoreState = RobotState.SCORE_L1;

    public enum RobotState {
        IDLE(IntakeState.IDLE, ManipulatorState.IDLE, ElevatorState.IDLE),
        INTAKE_ALGAE(IntakeState.INTAKE, ManipulatorState.IDLE, ElevatorState.IDLE),
        INTAKE_CORAL(IntakeState.IDLE, ManipulatorState.INTAKE, ElevatorState.IDLE),
        SCORE_ALGAE(IntakeState.REVERSE, ManipulatorState.IDLE, ElevatorState.IDLE),
        
        // prepare states for different levels
        PREPARE_L1(IntakeState.IDLE, ManipulatorState.IDLE, ElevatorState.L1),
        PREPARE_L2(IntakeState.IDLE, ManipulatorState.IDLE, ElevatorState.L2),
        PREPARE_L3(IntakeState.IDLE, ManipulatorState.IDLE, ElevatorState.L3),
        PREPARE_L4(IntakeState.IDLE, ManipulatorState.IDLE, ElevatorState.L4),

        // scoring states for different levels
        SCORE_L1(IntakeState.IDLE, ManipulatorState.SCORE, ElevatorState.L1),
        SCORE_L2(IntakeState.IDLE, ManipulatorState.SCORE, ElevatorState.L2),
        SCORE_L3(IntakeState.IDLE, ManipulatorState.SCORE, ElevatorState.L3),
        SCORE_L4(IntakeState.IDLE, ManipulatorState.SCORE, ElevatorState.L4),
        
        // special state
        KNOCK_ALGAE(IntakeState.IDLE, ManipulatorState.KNOCK, ElevatorState.KNOCK);

        public final IntakeState intakeState;
        public final ManipulatorState manipulatorState;
        public final ElevatorState elevatorState;

        private RobotState(IntakeState intakeState, ManipulatorState manipulatorState, ElevatorState elevatorState) {
            this.intakeState = intakeState;
            this.manipulatorState = manipulatorState;
            this.elevatorState = elevatorState;
        }
    }

    private RobotState state;
    private final Intake intake;
    private final CoralManipulator manipulator;
    private final Elevator elevator;

    public StateMachine(Intake intake, CoralManipulator manipulator, Elevator elevator) {
        this.intake = intake;
        this.manipulator = manipulator;
        this.elevator = elevator;
        this.state = RobotState.IDLE;

        stateTable = NetworkTableInstance.getDefault().getTable("StateMachine");
        currentStatePub = stateTable.getStringTopic("CurrentState").publish();
        intakeStatePub = stateTable.getStringTopic("SubsystemStates/Intake").publish();
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
                    (state.intakeState != IntakeState.REVERSE && 
                    state.intakeState != IntakeState.INTAKE) ? 
                       intake.changeState(newState.intakeState) : 
                       Commands.none()
                   ), Commands.sequence(
                    manipulator.changeState(newState.manipulatorState)
                        .until(() -> !manipulator.hasCoral())
                        .andThen(Commands.waitTime(Seconds.of(0.5)))
                        .andThen(changeState(RobotState.IDLE))
                )
            );   
        } 
        return Commands.sequence(
            Commands.runOnce(() -> {
                state = newState;
                System.out.println(newState.toString() + " scheduled");
            }),
            Commands.parallel(
                elevator.changeState(newState.elevatorState),
                manipulator.changeState(newState.manipulatorState),
                intake.changeState(newState.intakeState)
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

    private void publishStates() {
        currentStatePub.set(state.toString());
        elevatorStatePub.set(state.elevatorState.toString());
        intakeStatePub.set(state.intakeState.toString());
        manipulatorStatePub.set(state.manipulatorState.toString());
    }

    public void setTargetScoreState(RobotState targetState) {
        if (isScoreState(targetState)) {
            this.targetScoreState = targetState;
            System.out.println("Target score state set to: " + targetState);
        }
    }


}