package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SysIdRoutine driveSysIdRoutine;
    public SysIdRoutine steerSysIdRoutine;

    private final PhotonCamera leftCamera;
    private List<PhotonPipelineResult> leftCameraResults;
    private final PhotonCamera rightCamera;
    private List<PhotonPipelineResult> rightCameraResults;
    private final Map<PhotonCamera, Map<AlignmentPosition, Double>> targetPositions;

    private final PhotonPoseEstimator photonPoseEstimatorLeft;
    private final PhotonPoseEstimator photonPoseEstimatorRight;
    private final PIDController alignmentPID;

    private AlignmentPosition currentAlignmentPosition = AlignmentPosition.CENTER;
    private double lastValidCenterX = 0;
    private double lastDesiredCenterX = 0;
    private int noCamLoops = 0;
    private static final double ALIGNMENT_TOLERANCE = 20; //pixels
    
    private final NetworkTable table;
    private final StringPublisher alignmentPositionPub;
    private final BooleanPublisher isAlignedPub;
    private final DoublePublisher targetCenterXPub;
    private final DoublePublisher desiredXPub;

    private final DoublePublisher targetXPub;
    private final DoublePublisher targetYPub;
    private final DoublePublisher targetRotPub;
    private final Field2d field;

    private final DoublePublisher[] cancoderPubs;
    private final DoublePublisher[] anglePubs;
    private final DoublePublisher[] velocityPubs;

    private final Alert robotConfigAlert;
    private final Alert leftCameraAlert;
    private final Alert rightCameraAlert;

    public Swerve() {
        field = new Field2d();
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        alignmentPID = new PIDController(0.15, 0, 0); 
        alignmentPID.setTolerance(10, 10);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        //TODO: rename cameras
        final String lCamName = "Arducam_OV9782_USB_Camera";
        leftCamera = new PhotonCamera(lCamName);
        leftCameraAlert = new Alert(
            String.format("Left camera %s is not connected", lCamName), 
            AlertType.kError);
        final String rCamName = "Arducam_OV9281_USB_Camera";
        rightCamera = new PhotonCamera(rCamName);
        rightCameraAlert = new Alert(
            String.format("Right camera %s is not connected", rCamName), 
            AlertType.kError);
        targetPositions = Map.of(
            leftCamera, Constants.VisionConstants.leftCameraOffsets,
            rightCamera, Constants.VisionConstants.rightCameraOffsets
        );
        
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );
    
        final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        photonPoseEstimatorLeft = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.leftCamera 
        );

        photonPoseEstimatorRight = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.rightCamera
        );
        
        table = NetworkTableInstance.getDefault().getTable("Swerve");
        alignmentPositionPub = table.getStringTopic("Alignment/Position").publish();
        isAlignedPub = table.getBooleanTopic("Alignment/IsAligned").publish();
        targetCenterXPub = table.getDoubleTopic("Alignment/TargetCenterX").publish();
        desiredXPub = table.getDoubleTopic("Alignment/DesiredX").publish();
        targetXPub = table.getDoubleTopic("Alignment/OdomTarget/X").publish();
        targetYPub = table.getDoubleTopic("Alignment/OdomTarget/Y").publish();
        targetRotPub = table.getDoubleTopic("Alignment/OdomTarget/Rotation").publish();

        initializeScorePositions();

        cancoderPubs = new DoublePublisher[4];
        anglePubs = new DoublePublisher[4];
        velocityPubs = new DoublePublisher[4];
        for (int i = 0; i < 4; i++) {
            cancoderPubs[i] = table.getDoubleTopic("Module " + i + "/CANcoder").publish();
            anglePubs[i] = table.getDoubleTopic("Module " + i + "/Angle").publish();
            velocityPubs[i] = table.getDoubleTopic("Module " + i + "/Velocity").publish();
        }

        driveSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    // Apply the same voltage to all drive motors
                    for(SwerveModule mod : mSwerveMods){
                        mod.setDriveVoltage(volts.in(Volts));
                    }
                },
                null,
                this
            )
        );

        steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    // Apply the same voltage to all steer motors
                    for(SwerveModule mod : mSwerveMods){
                        mod.setSteerVoltage(volts.in(Volts));
                    }
                },
                null,
                this
            )
        );

        robotConfigAlert = new Alert(
            "Failed to get PathPlanner robot config from GUI settings, please ensure this file is present and restart the robot code", 
            AlertType.kError);
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
            robotConfigAlert.set(true);
            e.printStackTrace();
        }
    
        SmartDashboard.putData("Field", field);

        SmartDashboard.putData("Swerve Drive", new Sendable() {    
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () ->mSwerveMods[3].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
            }
        });
    }
    
    public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.dynamic(direction);
    }
    
    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return steerSysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return steerSysIdRoutine.dynamic(direction);
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
        return run(
            () -> {
                currentAlignmentPosition = position;
                PhotonCamera firstCam = leftCamera;
                PhotonCamera secondCam = rightCamera;
                List<PhotonPipelineResult> firstCamResults = leftCameraResults;
                List<PhotonPipelineResult> secondCamResults = rightCameraResults;

                if(position == AlignmentPosition.RIGHT) {
                    firstCam = rightCamera;
                    secondCam = leftCamera;
                    firstCamResults = rightCameraResults;
                    secondCamResults = leftCameraResults;
                }
                
                double actualCenterX = 0;
                double desiredCenterX = 0;
                double offset = 0;
                if(!firstCamResults.isEmpty() && firstCamResults.get(firstCamResults.size()-1).hasTargets()) {
                    var firstRes = firstCamResults.get(firstCamResults.size()-1);
                    if(!secondCamResults.isEmpty() && secondCamResults.get(secondCamResults.size()-1).hasTargets()) {
                        var secondRes = secondCamResults.get(secondCamResults.size()-1);
                        // Both cameras see so choose the camera with the least pose ambiguity 
                        if(secondRes.getBestTarget().getPoseAmbiguity() < firstRes.getBestTarget().getPoseAmbiguity()) {
                            actualCenterX = getCenterX(secondRes.getBestTarget().detectedCorners);
                            desiredCenterX = targetPositions.get(secondCam).get(position);
                        } else {
                            actualCenterX = getCenterX(firstRes.getBestTarget().detectedCorners);
                            desiredCenterX = targetPositions.get(firstCam).get(position);
                        }
                    } else { // Only camera with priority sees
                        actualCenterX = getCenterX(firstRes.getBestTarget().detectedCorners);
                        desiredCenterX = targetPositions.get(firstCam).get(position);
                    }
                    lastValidCenterX = actualCenterX;
                    lastDesiredCenterX = desiredCenterX;
                    noCamLoops = 0;
                } else if(!secondCamResults.isEmpty() && secondCamResults.get(secondCamResults.size()-1).hasTargets()) {
                    // Only second camera sees
                    var secondRes = secondCamResults.get(secondCamResults.size()-1);
                    actualCenterX = getCenterX(secondRes.getBestTarget().detectedCorners);
                    desiredCenterX = targetPositions.get(secondCam).get(position);
                    lastValidCenterX = actualCenterX;
                    lastDesiredCenterX = desiredCenterX;
                    noCamLoops = 0;
                } else { 
                    actualCenterX = lastValidCenterX; // No cameras see so use last valid center
                    desiredCenterX = lastDesiredCenterX;
                    noCamLoops++;
                }

                offset = desiredCenterX - actualCenterX;
                targetCenterXPub.set(actualCenterX);
                desiredXPub.set(desiredCenterX);

                double correction = alignmentPID.calculate(actualCenterX, desiredCenterX);
                double maxSpeed = 0.4;
                correction = Math.max(-maxSpeed, Math.min(maxSpeed, correction));
                if (Math.abs(offset) < ALIGNMENT_TOLERANCE) { //deadband so it doesn't keep correcting
                    correction = 0;
                }
            
                if (noCamLoops <= 5) {
                    drive(
                        new Translation2d(0, correction),
                        0,
                        false,
                        false
                    );
                } else {
                    drive(new Translation2d(), 0, false, false);
                }
            }
        ).until(() -> isAligned(position)).withTimeout(5);
    }

    /*
     * Checks if robot is aligned with the target and switches cameras if no targets in view.
     */
    private boolean isAligned(AlignmentPosition position) {
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
    }

    private boolean isAprilTagVisible() {
        return !leftCameraResults.isEmpty() && leftCameraResults.get(leftCameraResults.size() - 1).hasTargets() ||
            !rightCameraResults.isEmpty() && rightCameraResults.get(rightCameraResults.size() - 1).hasTargets();
    }
    
    public enum ReefScorePosition {
        FRONT,
        FRONTLEFT,
        BACKLEFT,
        BACK,
        BACKRIGHT,
        FRONTRIGHT
    }
    
    record ReefPositions(Map<Pose2d, ReefScorePosition> locations, List<Pose2d> poses) {};
    private ReefPositions scorePositions;
    
    private void initializeScorePositions() {
        Distance distanceAway = Inches.of(17.0);
        var alliance = DriverStation.getAlliance();
        boolean isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        ArrayList<Pose2d> scorePositionsList = new ArrayList<>();
        Map<Pose2d, ReefScorePosition> locations = new HashMap<>();
        
        if (!isRedAlliance) {            
            // Tag 18 (FRONT)
            Pose2d blueTag18 = new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(180));
            final var blueTag18Robot = new Pose2d(
                blueTag18.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(180)), 
                blueTag18.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(180)), 
                blueTag18.getRotation()
            );
            scorePositionsList.add(blueTag18Robot);
            locations.put(blueTag18Robot, ReefScorePosition.FRONT);
            
            // Tag 19 (FRONTLEFT)
            Pose2d blueTag19 = new Pose2d(4.0739, 4.7455, Rotation2d.fromDegrees(120));
            final var blueTag19Robot = new Pose2d(
                blueTag19.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(120)), 
                blueTag19.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(120)), 
                blueTag19.getRotation()
            );
            scorePositionsList.add(blueTag19Robot);
            locations.put(blueTag19Robot, ReefScorePosition.FRONTLEFT);
            
            // Tag 20 (BACKLEFT)
            Pose2d blueTag20 = new Pose2d(4.9047, 4.7455, Rotation2d.fromDegrees(60));
            final var blueTag20Robot = new Pose2d(
                    blueTag20.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(60)), 
                    blueTag20.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(60)), 
                    blueTag20.getRotation()
                );
            scorePositionsList.add(blueTag20Robot);
            locations.put(blueTag20Robot, ReefScorePosition.BACKLEFT);
            
            // Tag 21 (BACK)
            Pose2d blueTag21 = new Pose2d(5.3210, 4.0259, Rotation2d.fromDegrees(0));
            final var blueTag21Robot = new Pose2d(
                    blueTag21.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(0)), 
                    blueTag21.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(0)), 
                    blueTag21.getRotation()
                );
            scorePositionsList.add(blueTag21Robot);
            locations.put(blueTag21Robot, ReefScorePosition.BACK);
            
            // Tag 22 (BACKRIGHT)
            Pose2d blueTag22 = new Pose2d(4.9047, 3.3063, Rotation2d.fromDegrees(-60));
            final var blueTag22Robot = new Pose2d(
                    blueTag22.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-60)), 
                    blueTag22.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-60)), 
                    blueTag22.getRotation()
                );
            scorePositionsList.add(blueTag22Robot);
            locations.put(blueTag22Robot, ReefScorePosition.BACKRIGHT);
            
            // Tag 17 (FRONTRIGHT)
            Pose2d blueTag17 = new Pose2d(4.0739, 3.3063, Rotation2d.fromDegrees(-120));
            final var blueTag17Robot = new Pose2d(
                    blueTag17.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-120)), 
                    blueTag17.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-120)), 
                    blueTag17.getRotation()
                );
            scorePositionsList.add(blueTag17Robot);
            locations.put(blueTag17Robot, ReefScorePosition.FRONTRIGHT);
        } else { // Red Alliance
            // Tag 7 (FRONT)
            Pose2d redTag7 = new Pose2d(13.8905, 4.0259, Rotation2d.fromDegrees(0));
            final var redTag7Robot = new Pose2d(
                    redTag7.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(0)), 
                    redTag7.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(0)), 
                    redTag7.getRotation()
                );
            scorePositionsList.add(redTag7Robot);
            locations.put(redTag7Robot, ReefScorePosition.FRONT);
            
            // Tag 6 (FRONTLEFT)
            Pose2d redTag6 = new Pose2d(13.4744, 3.3063, Rotation2d.fromDegrees(-60));
            final var redTag6Robot = new Pose2d(
                    redTag6.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-60)), 
                    redTag6.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-60)), 
                    redTag6.getRotation()
                );
            scorePositionsList.add(redTag6Robot);
            locations.put(redTag6Robot, ReefScorePosition.FRONTLEFT);
            
            // Tag 11 (BACKLEFT)
            Pose2d redTag11 = new Pose2d(12.6434, 3.3063, Rotation2d.fromDegrees(-120));
            final var redTag11Robot = new Pose2d(
                    redTag11.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-120)), 
                    redTag11.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-120)), 
                    redTag11.getRotation()
                );
            scorePositionsList.add(redTag11Robot);
            locations.put(redTag11Robot, ReefScorePosition.BACKLEFT);
            
            // Tag 10 (BACK)
            Pose2d redTag10 = new Pose2d(12.2273, 4.0259, Rotation2d.fromDegrees(180));
            final var redTag10Robot = new Pose2d(
                    redTag10.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(180)), 
                    redTag10.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(180)), 
                    redTag10.getRotation()
                );
            scorePositionsList.add(redTag10Robot);
            locations.put(redTag10Robot, ReefScorePosition.BACK);
            
            // Tag 9 (BACKRIGHT)
            Pose2d redTag9 = new Pose2d(12.6434, 4.7455, Rotation2d.fromDegrees(120));
            final var redTag9Robot = new Pose2d(
                    redTag9.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(120)), 
                    redTag9.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(120)), 
                    redTag9.getRotation()
                );
            scorePositionsList.add(redTag9Robot);
            locations.put(redTag9Robot, ReefScorePosition.BACKRIGHT);
            
            // Tag 8 (FRONTRIGHT)
            Pose2d redTag8 = new Pose2d(13.4744, 4.7455, Rotation2d.fromDegrees(60));
            final var redTag8Robot = new Pose2d(
                    redTag8.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(60)), 
                    redTag8.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(60)), 
                    redTag8.getRotation()
                );
            scorePositionsList.add(redTag8Robot);
            locations.put(redTag8Robot, ReefScorePosition.FRONTRIGHT);
        }
        scorePositions = new ReefPositions(locations, scorePositionsList);
    }

    public Command runTrajectoryOdomAlign() {
        // find closest score position to align with
        final var pose = getPose();
        Pose2d targetPose = pose.nearest(scorePositions.poses);
        Pose2d currentPose = getPose();
        targetXPub.set(targetPose.getX());
        targetYPub.set(targetPose.getY());
        targetRotPub.set(targetPose.getRotation().getDegrees());

        // trajectory configuration
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond * 0.5, //TODO: tweak
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.5)
            .setKinematics(Constants.Swerve.swerveKinematics);
        
        // generate simple path with just start and end points
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            currentPose,
            List.of(), 
            targetPose,
            config);
        
        PIDController xController = new PIDController(Constants.AutoConstants.kPTranslationController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPTranslationController, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
            ));
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            this::getPose,
            Constants.Swerve.swerveKinematics,
            xController,
            yController,
            thetaController,
            this::setModuleStates,
            this);
        
        // resets odometry to the starting pose, follows trajectory, and stops robot
        return Commands.sequence(
            runOnce(() -> {
                table.getDoubleTopic("Alignment/OdomTarget/X").publish().set(targetPose.getX());
                table.getDoubleTopic("Alignment/OdomTarget/Y").publish().set(targetPose.getY());
                table.getDoubleTopic("Alignment/OdomTarget/Rotation").publish().set(targetPose.getRotation().getDegrees());
            }),
            swerveControllerCommand,
            runOnce(() -> drive(new Translation2d(), 0, true, true))
        );
    }

    public Command runAlignment() {
        return runOnce(() -> 
            Commands.either(
                runAutoAlign(AlignmentPosition.CENTER),
                runTrajectoryOdomAlign(),
                this::isAprilTagVisible
            )
        );
    }    

    @Override
    public void periodic(){
        leftCameraAlert.set(!leftCamera.isConnected());
        rightCameraAlert.set(!rightCamera.isConnected());
        leftCameraResults = leftCamera.getAllUnreadResults();
        rightCameraResults = rightCamera.getAllUnreadResults();
        poseEstimator.update(getGyroYaw(), getModulePositions());

        // TODO re-enable at least one camera after verifying no memory issues
        // if (!rightCameraResults.isEmpty()) {
        //     var photonRightUpdate = photonPoseEstimatorRight.update(rightCameraResults.get(rightCameraResults.size() - 1));
        //     if (photonRightUpdate.isPresent()) {
        //         EstimatedRobotPose visionPose = photonRightUpdate.get();
        //         poseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
        //     }
        // }
        // if (!leftCameraResults.isEmpty()) {
        //     var photonLeftUpdate = photonPoseEstimatorLeft.update(leftCameraResults.get(leftCameraResults.size() - 1));
        //     if (photonLeftUpdate.isPresent()) {
        //         EstimatedRobotPose visionPose = photonLeftUpdate.get();
        //         poseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
        //     }
        // }

        Pose2d currentPose = getPose();
        field.setRobotPose(currentPose);

        alignmentPositionPub.set(currentAlignmentPosition.toString());
        isAlignedPub.set(isAligned(currentAlignmentPosition));

        for(SwerveModule mod : mSwerveMods){
            cancoderPubs[mod.moduleNumber].set(mod.getCANcoder().getDegrees());
            anglePubs[mod.moduleNumber].set(mod.getPosition().angle.getDegrees());
            velocityPubs[mod.moduleNumber].set(mod.getState().speedMetersPerSecond);
        }
    }

    
}