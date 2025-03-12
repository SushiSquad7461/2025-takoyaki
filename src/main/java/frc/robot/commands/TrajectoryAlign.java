package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AlignmentPosition;
import frc.robot.util.AllianceUtil;
import frc.robot.util.ReefPositions;
import frc.robot.util.ReefPositions.ReefScorePosition;

import java.util.List;

public class TrajectoryAlign extends Command {
        
    private final Swerve swerve;
    private final Field2d field;
    private final AlignmentPosition alignmentPosition;
    private final ReefPositions.ReefPositionsMap scorePositions;
    Command cmd = Commands.none();

    public TrajectoryAlign(Swerve swerve, Field2d field, AlignmentPosition position) {
        this.swerve = swerve;
        this.field = field;
        this.alignmentPosition = position;
        ReefPositions.initialize();
        this.scorePositions = ReefPositions.getScorePositions();
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getPose();
        Pose2d targetPose = currentPose.nearest(scorePositions.poses());
        ReefScorePosition reefPosition = scorePositions.locations().get(targetPose);
        
        double offsetDistance = .165; //meters
        double xOffset = getHorizontalOffset(AllianceUtil.isRedAlliance(), offsetDistance);
        
        double[] offsets = calculatePositionalOffsets(reefPosition, xOffset);
        double offsetX = offsets[0];
        double offsetY = offsets[1];
        
        Pose2d offsetTargetPose = new Pose2d(
            targetPose.getX() + offsetX,
            targetPose.getY() + offsetY,
            targetPose.getRotation()
        );

        publishTargetInfo(currentPose, offsetTargetPose, offsetX, offsetY, targetPose);
        
        PathConstraints constraints = new PathConstraints(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond * 0.9, 
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.9,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond * 0.9,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared * 0.9
        );

        Translation2d displacement = offsetTargetPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d direction = new Rotation2d(displacement.getX(), displacement.getY());
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentPose.getTranslation(), direction),
            new Pose2d(offsetTargetPose.getTranslation(), direction)
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, targetPose.getRotation())
        );
        path.preventFlipping = true;
        
        field.getObject("target").setPose(targetPose);
        field.getObject("path").setPoses(path.getPathPoses());
        
        cmd = AutoBuilder.followPath(path);
        try {
            cmd.initialize();
        } catch(IndexOutOfBoundsException ex) {
            cmd = Commands.none();
        }
    }

    @Override
    public void execute() {
        cmd.execute();
    }
    
    @Override
    public boolean isFinished() {
        return cmd.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        field.getObject("target").setPose(new Pose2d(-100, 100, new Rotation2d()));
        field.getObject("path").setPoses(List.of());

        cmd.end(interrupted);
        cmd = Commands.none();
    }

    private double getHorizontalOffset(boolean isRedAlliance, double offsetDistance) {
        return switch (alignmentPosition) {
            case LEFT -> isRedAlliance ? -offsetDistance : offsetDistance;
            case RIGHT -> isRedAlliance ? offsetDistance : -offsetDistance;
            case CENTER -> 0.0;
        };
    }
    
    private double[] calculatePositionalOffsets(ReefScorePosition reefPosition, double xOffset) {
        double offsetX = 0;
        double offsetY = 0;
        
        switch(reefPosition) {
            case FRONT: // 180 degrees (horizontal, facing left)
                offsetX = 0;
                offsetY = xOffset;
                break;
            case FRONTLEFT: // 120 degrees
                offsetX = -xOffset * Math.sin(Math.toRadians(120));
                offsetY = xOffset * Math.cos(Math.toRadians(120));
                break;
            case BACKLEFT: // 60 degrees
                offsetX = -xOffset * Math.sin(Math.toRadians(60));
                offsetY = xOffset * Math.cos(Math.toRadians(60));
                break;
            case BACK: // 0 degrees (horizontal, facing right)
                offsetX = 0;
                offsetY = -xOffset;
                break;
            case BACKRIGHT: // -60 degrees
                offsetX = -xOffset * Math.sin(Math.toRadians(-60));
                offsetY = xOffset * Math.cos(Math.toRadians(-60));
                break;
            case FRONTRIGHT: // -120 degrees
                offsetX = -xOffset * Math.sin(Math.toRadians(-120));
                offsetY = xOffset * Math.cos(Math.toRadians(-120));
                break;
        }
        
        return new double[]{offsetX, offsetY};
    }
    
    private void publishTargetInfo(Pose2d currentPose, Pose2d offsetTargetPose, double offsetX, double offsetY, Pose2d targetRotation) {
        SmartDashboard.putNumber("Current X", currentPose.getX());
        SmartDashboard.putNumber("Current Y", currentPose.getY());
        SmartDashboard.putNumber("Offset Target X", offsetTargetPose.getX());
        SmartDashboard.putNumber("Offset Target Y", offsetTargetPose.getY());
        SmartDashboard.putNumber("X Offset Applied", offsetX);
        SmartDashboard.putNumber("Y Offset Applied", offsetY);
        SmartDashboard.putNumber("Target Rotation Applied", targetRotation.getRotation().getRadians());
    }
}