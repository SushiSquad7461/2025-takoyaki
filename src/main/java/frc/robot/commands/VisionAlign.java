package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AlignmentPosition;
import frc.robot.util.AllianceUtil;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionAlign extends Command {
    
    private final Swerve swerve;
    private final Field2d field;
    private final AlignmentPosition alignmentPosition;
    private final HashSet<Integer> reefIDs = new HashSet<>(Set.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11)); // The IDs of the AprilTags in the reefs
        
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    private double offsetDistance = 0.165; // meters
    
    private int bestReefID = -1;
    private boolean usingRightCamera = false;
    private PhotonTrackedTarget bestTarget = null;
    private AprilTagFieldLayout aprilTagFieldLayout;

    private final double ALIGNMENT_THRESHOLD_METERS = 0.02; //2cm
    private final double ROTATION_THRESHOLD_RADIANS = 0.02; //1 deg?
    
    private Transform2d scoringLocation; // in april tag coordinates
    
    public VisionAlign(Swerve swerve, Field2d field, AlignmentPosition position, AprilTagFieldLayout aprilTagFieldLayout) {
        this.swerve = swerve;
        this.field = field;
        this.alignmentPosition = position;
        this.aprilTagFieldLayout = aprilTagFieldLayout;

        this.xController = new PIDController(0.2, 0, 0); //p gains TODO: Prob bump these up
        this.yController = new PIDController(0.2, 0, 0);
        this.rotationController = new PIDController(0.2, 0, 0);
        
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        double xOffset = getHorizontalOffset(AllianceUtil.isRedAlliance(), offsetDistance);
        
        scoringLocation = new Transform2d(new Translation2d(0, -0.5), new Rotation2d(0));
        scoringLocation = new Transform2d(
            new Translation2d(scoringLocation.getX() + xOffset, scoringLocation.getY()),
            scoringLocation.getRotation()
        );
        
        SmartDashboard.putString("Vision Align Status", "Initialized");
    }
    
    @Override
    public void execute() {
        findBestAprilTag();
        
        if (bestReefID == -1 || bestTarget == null) {
            swerve.drive(new Translation2d(0, 0), 0, true, true);
            SmartDashboard.putString("Vision Align Status", "No valid target found");
            return;
        }
        
        // transform from camera to target
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
        
        Transform2d cameraToTarget2d = new Transform2d(
            new Translation2d(cameraToTarget.getX(), cameraToTarget.getY()),
            new Rotation2d(cameraToTarget.getRotation().getZ())
        );
        
        Transform2d cameraToRobot = usingRightCamera ? 
            Constants.VisionConstants.rightCamera2d : 
            Constants.VisionConstants.leftCamera2d;
        
        // robot pose in AprilTag coordinate
        Transform2d robotToTarget = calculateRobotToTargetTransform(cameraToTarget2d, cameraToRobot);
        
        // diff btw current position and desired scoring location to get error
        Transform2d error = calculateError(robotToTarget, scoringLocation);
        Rotation2d tagRotation = aprilTagFieldLayout.getTagPose(bestReefID).get().getRotation().toRotation2d();

        // multiply the error by a p gain to get the x and y velocity and the angular velocity
        double xVelocity = xController.calculate(error.getX(), 0);
        double yVelocity = yController.calculate(error.getY(), 0);
        double rotVelocity = rotationController.calculate(error.getRotation().getRadians(), 0);
        
        xVelocity = limit(xVelocity, Constants.AutoConstants.kMaxSpeedMetersPerSecond * 0.5);
        yVelocity = limit(yVelocity, Constants.AutoConstants.kMaxSpeedMetersPerSecond * 0.5);
        rotVelocity = limit(rotVelocity, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond * 0.5);
        
        Translation2d fieldVelocity = convertToFieldFrame(new Translation2d(xVelocity, yVelocity), tagRotation);
        swerve.drive(fieldVelocity, rotVelocity, true, true);
        publishDebugInfo(robotToTarget, error, fieldVelocity, rotVelocity);
    }
    
    private void findBestAprilTag() {
        bestReefID = -1;
        bestTarget = null;
        usingRightCamera = false;
        double lowestAmbiguity = Double.POSITIVE_INFINITY;
        
        // left cam check
        var resultLeft = swerve.leftCamera.getLatestResult();
        if (resultLeft.hasTargets()) {
            List<PhotonTrackedTarget> leftTargets = resultLeft.getTargets();
            
            for (PhotonTrackedTarget target : leftTargets) {
                if (reefIDs.contains(target.getFiducialId())) {
                    if (target.getPoseAmbiguity() < lowestAmbiguity && target.getPoseAmbiguity() != -1) {
                        bestReefID = target.getFiducialId();
                        lowestAmbiguity = target.getPoseAmbiguity();
                        bestTarget = target;
                        usingRightCamera = false;
                    }
                }
            }
        }
        
        // right cam check
        var resultRight = swerve.rightCamera.getLatestResult();
        if (resultRight.hasTargets()) {
            List<PhotonTrackedTarget> rightTargets = resultRight.getTargets();
            
            for (PhotonTrackedTarget target : rightTargets) {
                if (reefIDs.contains(target.getFiducialId())) {
                    if (target.getPoseAmbiguity() < lowestAmbiguity && target.getPoseAmbiguity() != -1) {
                        bestReefID = target.getFiducialId();
                        lowestAmbiguity = target.getPoseAmbiguity();
                        bestTarget = target;
                        usingRightCamera = true;
                    }
                }
            }
        }
        
        SmartDashboard.putNumber("Best Calc Reef ID", bestReefID);
        SmartDashboard.putBoolean("Using Right Camera", usingRightCamera);
        SmartDashboard.putNumber("Target Ambiguity", lowestAmbiguity);
    }
    
    /*
     * get the robot pose in the April tag centric frame (convert the robot to camera transformation to April tag centric frame by 
     * multiplying the xy by a rotation matrix derived from negative yaw angle, then subtract that transformation from cameraToTarget
     * to get the robot pose in the April tag reference frame)
     */
    private Transform2d calculateRobotToTargetTransform(Transform2d cameraToTarget2d, Transform2d cameraToRobot) {
        Transform2d robotToCamera = cameraToRobot.inverse();
        
        double targetYaw = cameraToTarget2d.getRotation().getRadians();
        double cosYaw = Math.cos(-targetYaw);
        double sinYaw = Math.sin(-targetYaw);
        
        double robotToCameraX = robotToCamera.getX();
        double robotToCameraY = robotToCamera.getY();
        
        double rotatedX = robotToCameraX * cosYaw - robotToCameraY * sinYaw;
        double rotatedY = robotToCameraX * sinYaw + robotToCameraY * cosYaw;
        
        Transform2d rotatedRobotToCamera = new Transform2d(
            new Translation2d(rotatedX, rotatedY),
            robotToCamera.getRotation()
        );
        
        double robotToTargetX = cameraToTarget2d.getX() - rotatedRobotToCamera.getX();
        double robotToTargetY = cameraToTarget2d.getY() - rotatedRobotToCamera.getY();
        Rotation2d robotToTargetRot = cameraToTarget2d.getRotation().minus(rotatedRobotToCamera.getRotation());
        
        return new Transform2d(new Translation2d(robotToTargetX, robotToTargetY), robotToTargetRot);
    }
    
    /*
     * scoring location minus robot pose (April tag frame) to get the error as a Transform2d
     */
    private Transform2d calculateError(Transform2d robotToTarget, Transform2d scoringLocation) {
        double errorX = scoringLocation.getX() - robotToTarget.getX();
        double errorY = scoringLocation.getY() - robotToTarget.getY();
        Rotation2d errorRot = scoringLocation.getRotation().minus(robotToTarget.getRotation());
        
        return new Transform2d(new Translation2d(errorX, errorY), errorRot);
    }

    /*
     * convert the x and y velocity back to field centric values by multiplying by a rotation 
     * matrix derived from negative April tag yaw angle (get this from the apriltagfield layout)
     */
    private Translation2d convertToFieldFrame(Translation2d tagFrameVelocity, Rotation2d tagRotation) {
        double negTagYaw = -tagRotation.getRadians();
        
        double cosYaw = Math.cos(negTagYaw);
        double sinYaw = Math.sin(negTagYaw);
        
        double fieldX = tagFrameVelocity.getX() * cosYaw - tagFrameVelocity.getY() * sinYaw;
        double fieldY = tagFrameVelocity.getX() * sinYaw + tagFrameVelocity.getY() * cosYaw;
        
        return new Translation2d(fieldX, fieldY);
    }
    
    private double getHorizontalOffset(boolean isRedAlliance, double offsetDistance) {
        return switch (alignmentPosition) {
            case LEFT -> isRedAlliance ? offsetDistance : -offsetDistance;
            case RIGHT -> isRedAlliance ? -offsetDistance : offsetDistance;
            case CENTER -> 0.0;
        };
    }
    
    private double limit(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    //TODO: when would this timeout if the tag is jittery
    @Override
    public boolean isFinished() {
        if (bestReefID == -1 || bestTarget == null) {
            return false;
        }
        
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
        
        Transform2d cameraToTarget2d = new Transform2d(
            new Translation2d(cameraToTarget.getX(), cameraToTarget.getY()),
            new Rotation2d(cameraToTarget.getRotation().getZ())
        );
        
        Transform2d cameraToRobot = usingRightCamera ? 
            Constants.VisionConstants.rightCamera2d : 
            Constants.VisionConstants.leftCamera2d;
        
        Transform2d robotToTarget = calculateRobotToTargetTransform(cameraToTarget2d, cameraToRobot);
        Transform2d error = calculateError(robotToTarget, scoringLocation);
        
        // Check if we're within the threshold for both position and rotation
        double positionError = Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY());
        double rotationError = Math.abs(error.getRotation().getRadians());
        
        return positionError <= ALIGNMENT_THRESHOLD_METERS && rotationError <= ROTATION_THRESHOLD_RADIANS;
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, true);
        SmartDashboard.putString("Vision Align Status", interrupted ? "interrupt" : "done");
    }
    
    private void publishDebugInfo(Transform2d robotToTarget, Transform2d error, Translation2d fieldVelocity, double rotVelocity) {
        SmartDashboard.putNumber("Robot To Tag X", robotToTarget.getX());
        SmartDashboard.putNumber("Robot To Tag Y", robotToTarget.getY());
        SmartDashboard.putNumber("Robot To Tag Rotation", robotToTarget.getRotation().getDegrees());
        
        SmartDashboard.putNumber("Error X", error.getX());
        SmartDashboard.putNumber("Error Y", error.getY());
        SmartDashboard.putNumber("Error Rotation", error.getRotation().getDegrees());
        
        SmartDashboard.putNumber("Field Velocity X", fieldVelocity.getX());
        SmartDashboard.putNumber("Field Velocity Y", fieldVelocity.getY());
        SmartDashboard.putNumber("Rotational Velocity", rotVelocity);
        
        double positionError = Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY());
        SmartDashboard.putNumber("Position Error", positionError);
        SmartDashboard.putNumber("Rotation Error", Math.abs(error.getRotation().getRadians()));
    }
}