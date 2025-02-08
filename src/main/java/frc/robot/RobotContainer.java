package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    public RobotContainer(){
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController operatorController = new CommandXboxController(1); 
        operatorController.a().onTrue(Commands.idle()); //Level 1
        operatorController.x().onTrue(Commands.idle()); //Level 2
        operatorController.y().onTrue(Commands.idle()); //Level 3
        operatorController.b().onTrue(Commands.idle()); //Level 4

        driverController.leftBumper().whileTrue(Commands.idle());  // intake wheels rolled in regular direction
        driverController.leftBumper().onFalse(Commands.idle());    // raise intake
        driverController.rightBumper().whileTrue(Commands.idle()); // intake wheels rolled in reverse
    
    }
}
