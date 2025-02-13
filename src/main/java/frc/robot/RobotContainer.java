// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AlignmentPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(Constants.Ports.DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(1); 

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final CoralManipulator manipulator = new CoralManipulator();
    private final Intake intake = new Intake();

    private final StateMachine stateMachine = new StateMachine(intake, manipulator, elevator);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        swerve.setDefaultCommand(new TeleopSwerve(
            swerve,
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX(), 
            () -> driverController.a().getAsBoolean()));
        
        driverController.y().onTrue(swerve.resetHeading());
        driverController.b().onTrue(swerve.runAutoAlign(AlignmentPosition.LEFT));
        driverController.leftTrigger().onTrue(swerve.runAutoAlign(AlignmentPosition.LEFT));
        driverController.rightTrigger().onTrue(swerve.runAutoAlign(AlignmentPosition.RIGHT));

        driverController.leftBumper().whileTrue(stateMachine.changeState(RobotState.INTAKE_ALGAE));  // intake wheels rolled in regular direction
        driverController.leftBumper().onFalse(stateMachine.changeState(RobotState.IDLE)); // raise intake
        driverController.rightBumper().whileTrue(stateMachine.changeState(RobotState.INTAKE_REVERSE)); // intake wheels rolled in reverse

        operatorController.a().onTrue(stateMachine.changeState(RobotState.SCORE_L1)); //spins manipulator
        operatorController.x().onTrue(stateMachine.changeState(RobotState.SCORE_L2));
        operatorController.y().onTrue(stateMachine.changeState(RobotState.SCORE_L3));
        operatorController.b().onTrue(stateMachine.changeState(RobotState.SCORE_L4));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(swerve);
    }
}