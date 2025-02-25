package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;
    private PhotonCamera activeCamera; // Tracks which camera is currently being used

    private final PhotonPoseEstimator photonPoseEstimator;
    private AlignmentPosition currentAlignmentPosition = AlignmentPosition.CENTER;
    private static final double LEFT_OFFSET = 380; //TODO: make these constants
    private static final double RIGHT_OFFSET = -340;
    private static final double LEFT_CAM_OFFSET = 100; //50px is about an inch
    private static final double RIGHT_CAM_OFFSET = -100;

    private static final double ALIGNMENT_TOLERANCE = 5;
    
    private final NetworkTable table;
    private final DoublePublisher poseXPub;
    private final DoublePublisher poseYPub;
    private final DoublePublisher poseRotPub;
    private final DoublePublisher[] cancoderPubs;
    private final DoublePublisher[] anglePubs;
    private final DoublePublisher[] velocityPubs;
    private final StringPublisher alignmentPositionPub;
    private final BooleanPublisher isAlignedPub;
    private final DoublePublisher targetCenterXPub;
    private final DoublePublisher desiredXPub;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule( 2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        //TODO: rename cameras
        leftCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        rightCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        activeCamera = leftCamera;
        
        photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new Transform3d() //adjust based on camera specific mounting
        );

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );

        table = NetworkTableInstance.getDefault().getTable("Swerve");
        poseXPub = table.getDoubleTopic("Pose/X").publish();
        poseYPub = table.getDoubleTopic("Pose/Y").publish();
        poseRotPub = table.getDoubleTopic("Pose/Rotation").publish();
        alignmentPositionPub = table.getStringTopic("Alignment/Position").publish();
        isAlignedPub = table.getBooleanTopic("Alignment/IsAligned").publish();
        targetCenterXPub = table.getDoubleTopic("Alignment/TargetCenterX").publish();
        desiredXPub = table.getDoubleTopic("Alignment/DesiredX").publish();

        cancoderPubs = new DoublePublisher[4];
        anglePubs = new DoublePublisher[4];
        velocityPubs = new DoublePublisher[4];
        
        for (int i = 0; i < 4; i++) {
            cancoderPubs[i] = table.getDoubleTopic("Module " + i + "/CANcoder").publish();
            anglePubs[i] = table.getDoubleTopic("Module " + i + "/Angle").publish();
            velocityPubs[i] = table.getDoubleTopic("Module " + i + "/Velocity").publish();
        }

        try{
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(Constants.AutoConstants.kPTranslationController, 0, 0), // Translation PID constants
                        new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this
            );
        } catch (Exception e) {
          e.printStackTrace();
        }
    
    }
    
    public static enum AlignmentPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public Command resetHeading() {
        return runOnce(() -> setPose(
            new Pose2d(
                getPose().getTranslation(),
                new Rotation2d()
            )
        ));
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    
    /*
     * Determines which camera to use based on position and vision quality
     * @param position The alignment position to use
     */
    private PhotonCamera getPreferredCamera(AlignmentPosition position) {
        PhotonCamera preferredCamera;
        
        switch(position) {
            case LEFT:
                preferredCamera = leftCamera;
                break;
            case RIGHT:
                preferredCamera = rightCamera;
                break;
            case CENTER:

            default:
                PhotonPipelineResult leftResult = leftCamera.getLatestResult();
                PhotonPipelineResult rightResult = rightCamera.getLatestResult();
                
                if (leftResult.hasTargets() && !rightResult.hasTargets()) {
                    preferredCamera = leftCamera;
                } else if (!leftResult.hasTargets() && rightResult.hasTargets()) {
                    preferredCamera = rightCamera;
                } else if (leftResult.hasTargets() && rightResult.hasTargets()) {
                    preferredCamera = (leftResult.getBestTarget().getPoseAmbiguity() 
                        < rightResult.getBestTarget().getPoseAmbiguity()) ? leftCamera : rightCamera;
                } else {
                    preferredCamera = activeCamera;
                }
                break;
        }
        
        //fallback to other camera if selected camera has no targets
        if (!preferredCamera.getLatestResult().hasTargets()) {
            PhotonCamera fallbackCamera = (preferredCamera == leftCamera) ? rightCamera : leftCamera;
            if (fallbackCamera.getLatestResult().hasTargets()) {
                return fallbackCamera;
            }
        }
        
        return preferredCamera;
    }
        
    /*
     * Aligns the robot to the target based on the given position
     */
    public Command runAutoAlign(AlignmentPosition position) {
        return new RunCommand(
            () -> {
                currentAlignmentPosition = position;
                
                PhotonCamera bestCamera = getPreferredCamera(position);
                PhotonPipelineResult result = bestCamera.getLatestResult();

                if (bestCamera != activeCamera) {
                    activeCamera = bestCamera;
                }
    
                if (result.hasTargets()) {
                    var bestTarget = result.getBestTarget();
                    List<TargetCorner> targets = bestTarget.getDetectedCorners();
                    double centerX = 0;

                    double idX = getTargetX(position);
                    for (var target : targets) {
                        centerX += target.x;
                    }
                    
                    centerX /= targets.size();
                    targetCenterXPub.set(centerX);
                    desiredXPub.set(idX);
                    table.getStringTopic("Alignment/ActiveCamera").publish().set(activeCamera == leftCamera ? "left camera" : "right camera");

                    //TODO: use pid loop here
                    if (Math.abs(centerX - idX) > ALIGNMENT_TOLERANCE) {
                        var dirMultiplier = centerX > idX ? -1 : 1;
                        drive(
                            new Translation2d(0, 0.25 * dirMultiplier),
                            0,
                            true,
                            false
                        );
                    }
                } else { //switch cameras if no targets found with getPreferredCamera()'s returned camera
                    activeCamera = (activeCamera == leftCamera) ? rightCamera : leftCamera;
                }
            }
        ).until(() -> isAligned(currentAlignmentPosition)).withTimeout(5);
    }

    //TODO: need to apply + calculate left camera positional offset from center and right positional offset from center in pixels
    private double getTargetX(AlignmentPosition position) {
        double centerX = Constants.Swerve.CAMERA_RESOLUTIONX / 2;
        var camOffset = activeCamera == leftCamera ? LEFT_CAM_OFFSET : RIGHT_CAM_OFFSET;
        return switch(position) {
            case LEFT -> centerX + LEFT_OFFSET + camOffset;
            case RIGHT -> centerX + RIGHT_OFFSET + camOffset;
            case CENTER -> centerX + camOffset;
        };
    }

    /*
     * Checks if robot is aligned with the target and switches cameras if no targets in view.
     */
    private boolean isAligned(AlignmentPosition position) {
        PhotonPipelineResult result = activeCamera.getLatestResult();

        if (!result.hasTargets()) {
            PhotonCamera otherCamera = (activeCamera == leftCamera) ? rightCamera : leftCamera;
            result = otherCamera.getLatestResult();

            if (result.hasTargets()) {
                activeCamera = otherCamera;
            } else {
                return false; //no targets found for both cameras
            }
        }

        var bestTarget = result.getBestTarget();
        List<TargetCorner> corners = bestTarget.getDetectedCorners();
        
        double centerX = 0;
        for (var corner : corners) {
            centerX += corner.x;
        }
        centerX /= corners.size();
        
        double targetX = getTargetX(position);
        table.getDoubleTopic("Alignment/Error").publish().set(Math.abs(centerX - targetX));
        return Math.abs(centerX - targetX) < ALIGNMENT_TOLERANCE;
    }

    @Override
    public void periodic(){
        poseEstimator.update(getGyroYaw(), getModulePositions());

        Pose2d currentPose = getPose();
        poseXPub.set(currentPose.getX());
        poseYPub.set(currentPose.getY());
        poseRotPub.set(currentPose.getRotation().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            cancoderPubs[mod.moduleNumber].set(mod.getCANcoder().getDegrees());
            anglePubs[mod.moduleNumber].set(mod.getPosition().angle.getDegrees());
            velocityPubs[mod.moduleNumber].set(mod.getState().speedMetersPerSecond);
        }

        alignmentPositionPub.set(currentAlignmentPosition.toString());
        isAlignedPub.set(isAligned(currentAlignmentPosition));    
    }

    
}