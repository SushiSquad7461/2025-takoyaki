package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    public RobotContainer(){
        CommandXboxController driverController = new CommandXboxController(1);
        CommandXboxController operatorController = new CommandXboxController(2); 
        Trigger xButton = operatorController.x(); 
        Trigger yButton = operatorController.y(); 
        Trigger aButton = operatorController.a(); 
        Trigger bButton = operatorController.b(); 
        Trigger leftBumper = driverController.leftBumper(); 
        Trigger rightBumper = driverController.rightBumper(); 
        

        aButton.onTrue(Commands.idle()); //Level 1
        xButton.onTrue(Commands.idle()); //Level 2
        yButton.onTrue(Commands.idle()); //Level 3
        bButton.onTrue(Commands.idle()); //Level 4

        leftBumper.whileTrue(Commands.idle());  // intake wheels rolled in regular direction
        leftBumper.onFalse(Commands.idle());    // raise intake
        rightBumper.whileTrue(Commands.idle()); // intake wheels rolled in reverse
    
    }
}
