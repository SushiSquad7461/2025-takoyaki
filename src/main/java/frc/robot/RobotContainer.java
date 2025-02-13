// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.AlignmentPosition;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController m_driverController = new CommandXboxController(Constants.Ports.DRIVER_PORT);

    /* Subsystems */
    private final Swerve swerve = new Swerve();


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
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getRightX(), 
            () -> m_driverController.a().getAsBoolean()));
        
        m_driverController.y().onTrue(swerve.resetHeading());
        m_driverController.b().onTrue(swerve.runAutoAlign(AlignmentPosition.LEFT));
        m_driverController.leftTrigger().onTrue(swerve.runAutoAlign(AlignmentPosition.LEFT));
        m_driverController.rightTrigger().onTrue(swerve.runAutoAlign(AlignmentPosition.RIGHT));
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