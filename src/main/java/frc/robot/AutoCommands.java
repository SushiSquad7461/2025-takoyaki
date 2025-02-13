package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private SendableChooser<Command> chooser;

    public AutoCommands(Swerve swerve, Elevator elevator, CoralManipulator manipulator) {

        // scoring commands for different levels: moves elevator to right position for scoring
        NamedCommands.registerCommand("prepareL2", 
            elevator.changeState(ElevatorState.L2)
        );

        NamedCommands.registerCommand("prepareL3", 
            elevator.changeState(ElevatorState.L3)
        );

        NamedCommands.registerCommand("prepareL4", 
            elevator.changeState(ElevatorState.L4)
        );

        // use if preload is loaded into manipulator
        NamedCommands.registerCommand("scoreManipulator", 
            manipulator.runRollers(Constants.CoralManipulator.SCORE_SPEED)
                .withTimeout(2.0)  // run rollers for X seconds
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

        chooser = new SendableChooser<>();
        chooser.setDefaultOption("Nothing", new InstantCommand());
        
        // TODO: need to make and name autos
        // blue alliance one piece autos
        chooser.addOption("B1 Score L1", makeAuto("B1_Score_L1"));
        chooser.addOption("B1 Score L2", makeAuto("B1_Score_L2"));
        chooser.addOption("B1 Score L3", makeAuto("B1_Score_L3"));
        chooser.addOption("B1 Score L4", makeAuto("B1_Score_L4"));

        chooser.addOption("B2 Score L1", makeAuto("B2_Score_L1"));
        chooser.addOption("B2 Score L2", makeAuto("B2_Score_L2"));
        chooser.addOption("B2 Score L3", makeAuto("B2_Score_L3"));
        chooser.addOption("B2 Score L4", makeAuto("B2_Score_L4"));

        chooser.addOption("B3 Score L1", makeAuto("B3_Score_L1"));
        chooser.addOption("B3 Score L2", makeAuto("B3_Score_L2"));
        chooser.addOption("B3 Score L3", makeAuto("B3_Score_L3"));
        chooser.addOption("B3 Score L4", makeAuto("B3_Score_L4"));
        
        // red alliance one piece autos
        chooser.addOption("B1 Score L1", makeAuto("B1_Score_L1"));
        chooser.addOption("R1 Score L2", makeAuto("R1_Score_L2"));
        chooser.addOption("R1 Score L3", makeAuto("R1_Score_L3"));
        chooser.addOption("R1 Score L4", makeAuto("R1_Score_L4"));

        chooser.addOption("B1 Score L1", makeAuto("B1_Score_L1"));
        chooser.addOption("R2 Score L2", makeAuto("R2_Score_L2"));
        chooser.addOption("R2 Score L3", makeAuto("R2_Score_L3"));
        chooser.addOption("R2 Score L4", makeAuto("R2_Score_L4"));

        chooser.addOption("B1 Score L1", makeAuto("B1_Score_L1"));
        chooser.addOption("R3 Score L2", makeAuto("R3_Score_L2"));
        chooser.addOption("R3 Score L3", makeAuto("R3_Score_L3"));
        chooser.addOption("R3 Score L4", makeAuto("R3_Score_L4"));

        SmartDashboard.putData("Auto Selector", chooser);
    }

    private Command makeAuto(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command getAuto() {
        return chooser.getSelected();
    }
}