package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AlignmentPosition;

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
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoCommands(Swerve swerve, Elevator elevator, CoralManipulator manipulator, StateMachine stateMachine) {
        autoTable = NetworkTableInstance.getDefault().getTable("Auto");
        selectedAutoPublisher = autoTable.getStringTopic("selectedAuto").publish();
        selectedAutoPublisher.set("Nothing");

        NamedCommands.registerCommand("prepareL1", 
            stateMachine.changeState(RobotState.PREPARE_L1)
        );

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
        
        NamedCommands.registerCommand("intakeCoral", 
            stateMachine.changeState(RobotState.INTAKE_CORAL)
        );

        NamedCommands.registerCommand("alignLeft", 
            swerve.runRoyalAlign(AlignmentPosition.LEFT)
        );
    
        NamedCommands.registerCommand("alignRight", 
            swerve.runRoyalAlign(AlignmentPosition.LEFT)
        );
    
    
        // reset state
        NamedCommands.registerCommand("resetToIdle",
            stateMachine.changeState(RobotState.IDLE)
        );

        autoChooser.setDefaultOption("Nothing", new InstantCommand());

        
        // // TODO: need to make autos
        // // one piece autos
        // for (int position = 1; position <= 3; position++) {
        //     for (int level = 1; level <= 4; level++) {
        //         String name = String.format("B%d_Score_L%d", position, level);
        //         autoChooser.addOption(name, makeAuto(name));
        //     }
        // }

        autoChooser.addOption("B1_Score_L2", makeAuto("B1_Score_L2"));
        autoChooser.addOption("B1_two_piece_L2", makeAuto("B1_two_piece_L2"));
        autoChooser.addOption("B1_three_piece_L2", makeAuto("B1_three_piece_L2"));

        autoChooser.addOption("Leaving_B2", makeAuto("Leaving_B2"));
        autoChooser.addOption("B2_Score_L2", makeAuto("B2_Score_L2"));
        //autoChooser.addOption("Leaving_Center", makeAuto("Leaving_Center"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private Command makeAuto(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }
}