package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;

    private final NetworkTable table;
    private final DoublePublisher poseXPub;
    private final DoublePublisher poseYPub;
    private final DoublePublisher poseRotPub;
    private final DoublePublisher[] cancoderPubs;
    private final DoublePublisher[] anglePubs;
    private final DoublePublisher[] velocityPubs;

    private final PIDController xController = new PIDController(0.0, 0, 0);
    private final PIDController yController = new PIDController(0.0, 0, 0);
    private final PIDController rotationController = new PIDController(0.0, 0, 0);

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        camera = new PhotonCamera("23");
        photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new Transform3d() //adjust based on camera mounting
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
        
        cancoderPubs = new DoublePublisher[4];
        anglePubs = new DoublePublisher[4];
        velocityPubs = new DoublePublisher[4];
        
        for (int i = 0; i < 4; i++) {
            cancoderPubs[i] = table.getDoubleTopic("Module " + i + "/CANcoder").publish();
            anglePubs[i] = table.getDoubleTopic("Module " + i + "/Angle").publish();
            velocityPubs[i] = table.getDoubleTopic("Module " + i + "/Velocity").publish();
        }
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

    public Command getAutoAlignCommand() {
        return new RunCommand(
            () -> {
                PhotonPipelineResult result = camera.getLatestResult();
                
                if (result.hasTargets()) {
                    var bestTarget = result.getBestTarget();
                    
                    // target pose relative to the camera
                    double x = bestTarget.getBestCameraToTarget().getX();
                    double y = bestTarget.getBestCameraToTarget().getY();
                    double yaw = bestTarget.getYaw();
                    
                    double xSpeed = xController.calculate(x, 0);
                    double ySpeed = yController.calculate(y, 0);
                    double rotationSpeed = rotationController.calculate(yaw, 0);
                    
                    // drive with the calculated speeds
                    drive(
                        new Translation2d(xSpeed, ySpeed),
                        rotationSpeed,
                        true,
                        false
                    );
                }
            }
        );
    }

    @Override
    public void periodic(){
        poseEstimator.update(getGyroYaw(), getModulePositions());

        Pose2d currentPose = getPose();
        table.getEntry("Pose/X").setDouble(currentPose.getX());
        table.getEntry("Pose/Y").setDouble(currentPose.getY());
        table.getEntry("Pose/Rotation").setDouble(currentPose.getRotation().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            table.getEntry("Module " + mod.moduleNumber + "/CANcoder").setDouble(mod.getCANcoder().getDegrees());
            table.getEntry("Module " + mod.moduleNumber + "/Angle").setDouble(mod.getPosition().angle.getDegrees());
            table.getEntry("Module " + mod.moduleNumber + "/Velocity").setDouble(mod.getState().speedMetersPerSecond);
        }
    }
}