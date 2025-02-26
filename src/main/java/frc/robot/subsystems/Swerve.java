package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

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
    private List<PhotonPipelineResult> leftCameraResults;
    private final PhotonCamera rightCamera;
    private List<PhotonPipelineResult> rightCameraResults;
    private final Map<PhotonCamera, Map<AlignmentPosition, Double>> targetPositions;

    private final PhotonPoseEstimator photonPoseEstimatorLeft;
    private final PhotonPoseEstimator photonPoseEstimatorRight;

    private AlignmentPosition currentAlignmentPosition = AlignmentPosition.CENTER;

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
        targetPositions = Map.of(
            leftCamera, Constants.VisionConstants.leftCameraOffsets,
            rightCamera, Constants.VisionConstants.rightCameraOffsets
        );
        
        photonPoseEstimatorLeft = new PhotonPoseEstimator(
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.leftTransform3d 
        );

        photonPoseEstimatorRight = new PhotonPoseEstimator(
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.rightTransform3d
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

    private static double getCenterX(List<TargetCorner> corners) {
        double centerX = 0;
        for (var corner : corners) {
            centerX += corner.x;
        }
        centerX /= corners.size();
        return centerX;
    }
        
    /*
     * Aligns the robot to the target based on the given position
     */
    public Command runAutoAlign(AlignmentPosition position) {
        

        return new RunCommand(
            () -> {
                currentAlignmentPosition = position;
                PhotonCamera firstCam = leftCamera;
                PhotonCamera secondCam = rightCamera;
                List<PhotonPipelineResult> firstCamResults = leftCameraResults;
                List<PhotonPipelineResult> secondCamResults = rightCameraResults;

                if(position == AlignmentPosition.RIGHT) {
                    firstCam = leftCamera;
                    secondCam = rightCamera;
                    firstCamResults = leftCameraResults;
                    secondCamResults = rightCameraResults;
                }
                
                double offset = 0;
                if(!firstCamResults.isEmpty() && firstCamResults.get(firstCamResults.size()-1).hasTargets()) {
                    var firstRes = firstCamResults.get(firstCamResults.size()-1);
                    if(!secondCamResults.isEmpty() && secondCamResults.get(secondCamResults.size()-1).hasTargets()) {
                        var secondRes = secondCamResults.get(secondCamResults.size()-1);
                        if(secondRes.getBestTarget().getPoseAmbiguity() < firstRes.getBestTarget().getPoseAmbiguity()) {
                            offset = getCenterX(secondRes.getBestTarget().detectedCorners) - targetPositions.get(secondCam).get(position);
                        } else {
                            offset = getCenterX(firstRes.getBestTarget().detectedCorners) - targetPositions.get(firstCam).get(position);
                        }
                    } else {
                        offset = getCenterX(firstRes.getBestTarget().detectedCorners) - targetPositions.get(firstCam).get(position);
                    }
                } else if(!secondCamResults.isEmpty() && secondCamResults.get(secondCamResults.size()-1).hasTargets()) {
                    var secondRes = secondCamResults.get(secondCamResults.size()-1);
                    offset = getCenterX(secondRes.getBestTarget().detectedCorners) - targetPositions.get(secondCam).get(position);
                }

                drive(
                    new Translation2d(0, 0.25 * Math.signum(offset)),
                    0,
                    true,
                    false
                );
            }
        ).until(isAligned(position)).withTimeout(5);
    }

    /*
     * Checks if robot is aligned with the target and switches cameras if no targets in view.
     */
    private BooleanSupplier isAligned(AlignmentPosition position) {
        return () -> {
            final PhotonCamera cam = position == AlignmentPosition.RIGHT
                ? rightCamera
                : leftCamera;
            final List<PhotonPipelineResult> results = position == AlignmentPosition.RIGHT
                ? rightCameraResults
                : leftCameraResults;
            final double targetAprilTagX = targetPositions.get(cam).get(position);
            if(results.isEmpty()) return false;
            var last = results.get(results.size() - 1);
            if(last.hasTargets()) {
                return Math.abs(getCenterX(last.getBestTarget().detectedCorners) - targetAprilTagX) < ALIGNMENT_TOLERANCE;
            }
            return false;
        };
    }

    @Override
    public void periodic(){
        leftCameraResults = leftCamera.getAllUnreadResults();
        rightCameraResults = rightCamera.getAllUnreadResults();

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
        isAlignedPub.set(isAligned(currentAlignmentPosition).getAsBoolean());
    }

    
}