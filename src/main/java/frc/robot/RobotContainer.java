// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AlignmentPosition;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(Constants.Ports.DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.Ports.OPERATOR_PORT);
    private final CommandXboxController programmerController = new CommandXboxController(Constants.Ports.PROG_PORT);
    
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final CoralManipulator manipulator = new CoralManipulator();
    private final Intake intake = new Intake();

    private final StateMachine stateMachine = new StateMachine(intake, manipulator, elevator);
    private final AutoCommands autos = new AutoCommands(swerve, elevator, manipulator, stateMachine);
    private Command[] scoreCommands = {
        stateMachine.changeState(RobotState.SCORE_L1),
        stateMachine.changeState(RobotState.SCORE_L2),
        stateMachine.changeState(RobotState.SCORE_L3),
        stateMachine.changeState(RobotState.SCORE_L4)
        // TODO add L4 when energy chain fixed
    };
    private Command targetScoreCommand = scoreCommands[0];
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        elevator.setDefaultCommand(elevator.resetElevator().andThen(() -> elevator.removeDefaultCommand()));
        intake.setDefaultCommand(intake.reset(true).andThen(() -> intake.removeDefaultCommand()));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        final var idle = stateMachine.changeState(RobotState.IDLE);

        swerve.setDefaultCommand(new TeleopSwerve(
            swerve,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(), 
            () -> driverController.a().getAsBoolean())); // allows you to drive as robot relative only while holding down the button
        
        // Driver handles robot positioning, alignment, and algae
        driverController.y().onTrue(swerve.resetHeading());
        driverController.a().onTrue(stateMachine.changeState(RobotState.INTAKE_CORAL)).onFalse(idle);
        driverController.b().whileTrue(swerve.runAutoAlign(AlignmentPosition.CENTER));
        driverController.leftTrigger().whileTrue(swerve.runAutoAlign(AlignmentPosition.LEFT));
        driverController.rightTrigger().whileTrue(swerve.runAutoAlign(AlignmentPosition.RIGHT));

        driverController.leftBumper().onTrue(stateMachine.changeState(RobotState.INTAKE_ALGAE)).onFalse(idle);  // intake wheels rolled in regular direction
        driverController.rightBumper().onTrue(stateMachine.changeState(RobotState.SCORE_ALGAE)).onFalse(idle); // intake wheels rolled in reverse

        // Operator controls coral scoring
        operatorController.leftBumper().onTrue(stateMachine.changeState(RobotState.INTAKE_CORAL)).onFalse(idle);
        operatorController.a().onTrue(Commands.runOnce(() -> targetScoreCommand = scoreCommands[0]));
        operatorController.x().onTrue(Commands.runOnce(() -> targetScoreCommand = scoreCommands[1]));
        operatorController.y().onTrue(Commands.runOnce(() -> targetScoreCommand = scoreCommands[2]));
        operatorController.b().onTrue(Commands.runOnce(() -> targetScoreCommand = scoreCommands[3]));

        //operatorController.rightBumper().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(targetScoreCommand))).onFalse(idle);
        //operatorController.rightTrigger().onTrue(manipulator.runRollers(0.1)).onFalse(Commands.runOnce(() -> manipulator.runRollers(0)));

        operatorController.rightBumper().onTrue(Commands.defer(() -> targetScoreCommand, Set.of(swerve, elevator, manipulator, intake))).onFalse(idle);
        operatorController.rightTrigger().onTrue(manipulator.runRollers(0.1)).onFalse(manipulator.runRollers(0));        
        
        // special state => override and resetting to idle, and knocking algae
        operatorController.back().onTrue(idle);
        operatorController.leftTrigger().onTrue(stateMachine.changeState(RobotState.KNOCK_ALGAE)).onFalse(idle);

        // odometry autoalign testing
        operatorController.povUp().whileTrue(swerve.runTrajectoryOdomAlign());
        operatorController.povDown().whileTrue(swerve.runTrajectoryOdomAlign());


        programmerController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        programmerController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        programmerController.a().and(programmerController.rightTrigger()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        programmerController.b().and(programmerController.rightTrigger()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        programmerController.x().and(programmerController.rightTrigger()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
        programmerController.y().and(programmerController.rightTrigger()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));

        programmerController.a().and(programmerController.leftTrigger()).whileTrue(intake.sysIdQuasistatic(Direction.kForward));
        programmerController.b().and(programmerController.leftTrigger()).whileTrue(intake.sysIdQuasistatic(Direction.kReverse));
        programmerController.x().and(programmerController.leftTrigger()).whileTrue(intake.sysIdDynamic(Direction.kForward));
        programmerController.y().and(programmerController.leftTrigger()).whileTrue(intake.sysIdDynamic(Direction.kReverse));

        programmerController.a().and(programmerController.povLeft()).whileTrue(swerve.sysIdDriveQuasistatic(Direction.kForward));
        programmerController.b().and(programmerController.povLeft()).whileTrue(swerve.sysIdDriveQuasistatic(Direction.kReverse));
        programmerController.x().and(programmerController.povLeft()).whileTrue(swerve.sysIdDriveDynamic(Direction.kForward));
        programmerController.y().and(programmerController.povLeft()).whileTrue(swerve.sysIdDriveDynamic(Direction.kReverse));

        programmerController.a().and(programmerController.povRight()).whileTrue(swerve.sysIdSteerQuasistatic(Direction.kForward));
        programmerController.b().and(programmerController.povRight()).whileTrue(swerve.sysIdSteerQuasistatic(Direction.kReverse));
        programmerController.x().and(programmerController.povRight()).whileTrue(swerve.sysIdSteerDynamic(Direction.kForward));
        programmerController.y().and(programmerController.povRight()).whileTrue(swerve.sysIdSteerDynamic(Direction.kReverse));

        programmerController.povUp().whileTrue(elevator.goUp());
        programmerController.povDown().whileTrue(elevator.goDown());
        programmerController.a().whileTrue(elevator.resetElevator());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autos.getAuto();
    }
}
