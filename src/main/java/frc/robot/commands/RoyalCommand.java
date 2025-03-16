// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ReefPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RoyalCommand extends Command {
  ProfiledPIDController thetaController;
  ProfiledPIDController xController;
  ProfiledPIDController yController;
  Swerve swerve;
  Pose2d currentPose;
  Pose2d targetPose;

  ReefPositions.ReefPositionsMap scorePositions;

  /** Creates a new RoyalCommand. */
  public RoyalCommand(Swerve swerve) {
    this.swerve = swerve;
    xController = new ProfiledPIDController(10, 0, 0, new Constraints(1, 3));
    yController = new ProfiledPIDController(10, 0, 0, new Constraints(1, 3));
    thetaController = new ProfiledPIDController(8, 0, 0, new Constraints(Math.PI, Math.PI));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = swerve.getPose();

    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
    Pose2d targetPose = currentPose.nearest(scorePositions.poses());

    xController.setGoal(targetPose.getX());
    xController.setGoal(targetPose.getY());
    xController.setGoal(MathUtil.angleModulus(targetPose.getRotation().getRadians()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    swerve.drive(
      new Translation2d(xController.calculate(currentPose.getX()), yController.calculate(currentPose.getY())), 
        thetaController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians())), 
        true, 
        false
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(Translation2d.kZero, 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentPose.getTranslation().minus(targetPose.getTranslation()).getNorm() < 0.0125 
    && Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 5;
  }
}
