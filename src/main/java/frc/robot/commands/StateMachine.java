package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ManipulatorState;
import frc.robot.subsystems.ElevatorState;
import frc.robot.subsystems.IntakeState;

public class StateMachine extends Command {
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;
    private final StringPublisher intakeStatePub;
    private final StringPublisher manipulatorStatePub;
    private final StringPublisher elevatorStatePub;

    public enum RobotState {
        IDLE(IntakeState.IDLE, ManipulatorState.IDLE, ElevatorState.IDLE),
        INTAKE_ALGAE(IntakeState.INTAKE, ManipulatorState.IDLE, ElevatorState.IDLE),
        INTAKE_CORAL(IntakeState.IDLE, ManipulatorState.INTAKE, ElevatorState.IDLE),
        INTAKE_REVERSE(IntakeState.REVERSE, ManipulatorState.IDLE, ElevatorState.IDLE),
        
        // scoring states for different levels
        SCORE_L1(IntakeState.IDLE, ManipulatorState.L1, ElevatorState.L1),
        SCORE_L2(IntakeState.IDLE, ManipulatorState.L1, ElevatorState.L2),
        SCORE_L3(IntakeState.IDLE, ManipulatorState.L3, ElevatorState.L3),
        SCORE_L4(IntakeState.IDLE, ManipulatorState.L4, ElevatorState.L4),
        
        // special states
        KNOCK_ALGAE(IntakeState.IDLE, ManipulatorState.KNOCK, ElevatorState.IDLE),
        CARRY_ALGAE(IntakeState.CARRYING, ManipulatorState.IDLE, ElevatorState.IDLE);

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

        publishStates();
    }

    @Override
    public void execute() {
        publishStates();

        // returning to idle after scoring
        if (isScoreState(state) && !manipulator.hasCoral()) {
            scheduleNewState(RobotState.IDLE);
        }
    }

    public void scheduleNewState(RobotState newState) {
        changeState(newState).schedule();
    }

    public Command changeState(RobotState newState) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                state = newState;
                System.out.println(newState.toString() + " scheduled");
                publishStates();
            }),
            // first handle elevator state to avoid collisions
            elevator.changeState(newState.elevatorState),
            // next, change manipulator and intake states in parallel
            Commands.parallel(
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

    private void publishStates() {
        currentStatePub.set(state.toString());
        intakeStatePub.set(state.intakeState.toString());
        manipulatorStatePub.set(state.manipulatorState.toString());
        elevatorStatePub.set(state.elevatorState.toString());
    }

}