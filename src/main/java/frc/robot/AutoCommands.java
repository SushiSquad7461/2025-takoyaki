package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorState;
import frc.robot.subsystems.ManipulatorState;
import frc.robot.subsystems.Swerve;

/* Notes for Auto Maker:
 * Start at B1/B2/B3 position with preloaded game piece
 * Add event marker "prepareL3" as robot starts moving
 * Once in scoring position, add event marker "scoreManipulator"
 * ONE PIECE END => End with "resetToIdle." Continue reading for TWO PIECE.
 * At coral station, add event marker "runManipulator"
 * Include a wait time of ~0.5-1.0 seconds to ensure intake
 * Once coral is acquired (beambreak triggered), path to scoring position
 * Add event marker "prepareL3" during return movement
 * Add event marker "scoreManipulator" when in position
 * End with "resetToIdle"
 */

public class AutoCommands {
    private final NetworkTable autoTable;
    private final StringPublisher selectedAutoPublisher;
    private final Map<String, Command> autoCommands = new HashMap<>();
    private String selectedAuto = "Nothing";

    public AutoCommands(Swerve swerve, Elevator elevator, CoralManipulator manipulator, StateMachine stateMachine) {
        autoTable = NetworkTableInstance.getDefault().getTable("Auto");
        selectedAutoPublisher = autoTable.getStringTopic("selectedAuto").publish();
        selectedAutoPublisher.set("Nothing");

        // prepare commands for different levels: moves elevator to right position for scoring
        NamedCommands.registerCommand("prepareL2", 
            stateMachine.changeState(RobotState.PREPARE_L2)
        );

        NamedCommands.registerCommand("prepareL3", 
            stateMachine.changeState(RobotState.PREPARE_L3)
        );

        NamedCommands.registerCommand("prepareL4", 
            stateMachine.changeState(RobotState.PREPARE_L4)
        );

        NamedCommands.registerCommand("scoreL1", 
            stateMachine.changeState(RobotState.SCORE_L1)
        );

        NamedCommands.registerCommand("scoreL2", 
            stateMachine.changeState(RobotState.SCORE_L2)
        );

        NamedCommands.registerCommand("scoreL3", 
            stateMachine.changeState(RobotState.SCORE_L3)
        );

        NamedCommands.registerCommand("scoreL4", 
            stateMachine.changeState(RobotState.SCORE_L4)
        );

        // use if preload is loaded into manipulator
        NamedCommands.registerCommand("scoreManipulator", 
            manipulator.runRollers(Constants.CoralManipulator.SCORE_SPEED)
                .withTimeout(2.0)  // run rollers for X seconds
                .andThen(manipulator.stopRollers())
        );

        NamedCommands.registerCommand("triggerScore", 
            manipulator.runRollers(Constants.CoralManipulator.SCORE_SPEED)
                .withTimeout(2.0)
                .andThen(manipulator.stopRollers())
        );

        // use if preload is loaded into intake manipulator handoff OR trying to intake from hp station
        NamedCommands.registerCommand("runManipulator",
            manipulator.runRollers(Constants.CoralManipulator.INTAKE_SPEED)
                .until(manipulator::hasCoral)  // run until beam break detects coral
                .withTimeout(5.0)  // timeout after 5 seconds if no coral detected
                .andThen(manipulator.stopRollers())
        );

        // algae intake commands (TODO: expand on intake functionalities)
        NamedCommands.registerCommand("prepareAlgaeIntake",
            elevator.changeState(ElevatorState.IDLE)  // grounding elevator bec intake
        );

        NamedCommands.registerCommand("knockAlgae",
            elevator.changeState(ElevatorState.KNOCK)
            .andThen(manipulator.changeState(ManipulatorState.KNOCK))
        );
    
        // reset state
        NamedCommands.registerCommand("resetToIdle",
            elevator.changeState(ElevatorState.IDLE)
                .andThen(manipulator.stopRollers())
        );

        registerAutoCommand("Nothing", new InstantCommand());

        
        // TODO: need to make autos
        // one piece autos
        for (int position = 1; position <= 3; position++) {
            for (int level = 1; level <= 4; level++) {
                String name = String.format("B%d_Score_L%d", position, level);
                registerAutoCommand(name, makeAuto(name));
            }
        }

        var autoSubscriber = autoTable.getStringTopic("selectedAuto").subscribe("Nothing");
        selectedAuto = autoSubscriber.get();
    }

    private void registerAutoCommand(String name, Command command) {
        autoCommands.put(name, command);
        autoTable.getStringArrayTopic("availableAutos").publish().set(
            autoCommands.keySet().toArray(new String[0])
        );
    }

    private Command makeAuto(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command getAuto() {
        return autoCommands.getOrDefault(selectedAuto, new InstantCommand());
    }
}