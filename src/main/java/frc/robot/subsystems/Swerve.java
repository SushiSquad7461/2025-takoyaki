package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.util.AprilTagFields;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    private PhotonPoseEstimator photonPoseEstimatorLeft = new PhotonPoseEstimator(null, null, null);
    private PhotonPoseEstimator photonPoseEstimatorRight = new PhotonPoseEstimator(null, null, null);

    private AlignmentPosition currentAlignmentPosition = AlignmentPosition.CENTER;

    private static final double ALIGNMENT_TOLERANCE = 5; //pixels
    
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
    private final StringPublisher selectedPositionPub;

    private static final AprilTagFields WELDED_LAYOUT = AprilTagFields.k2025Reefscape;
    private static final AprilTagFields ANDYMARK_LAYOUT = AprilTagFields.k2025ReefscapeAndymark;
    private String currentFieldLayout = WELDED_LAYOUT.toString();
    private final StringPublisher fieldLayoutPub;
    private final StringArrayPublisher fieldLayoutOptionsPub;
    private int fieldLayoutListener;

    private ReefScorePosition selectedScorePosition;

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
        rightCamera = new PhotonCamera("Arducam_OV9782_USB_Camera");
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

        try {
            AprilTagFieldLayout weldedLayout = AprilTagFieldLayout.loadFromResource(WELDED_LAYOUT.m_resourceFile);
            photonPoseEstimatorLeft = new PhotonPoseEstimator(
                weldedLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.VisionConstants.leftTransform3d 
            );

            photonPoseEstimatorRight = new PhotonPoseEstimator(
                weldedLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.VisionConstants.rightTransform3d
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
        
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

        selectedPositionPub = table.getStringTopic("AutoAlign/SelectedPosition").publish();
        selectedScorePosition = ReefScorePosition.FRONT;

        for (int i = 0; i < 4; i++) {
            cancoderPubs[i] = table.getDoubleTopic("Module " + i + "/CANcoder").publish();
            anglePubs[i] = table.getDoubleTopic("Module " + i + "/Angle").publish();
            velocityPubs[i] = table.getDoubleTopic("Module " + i + "/Velocity").publish();
        }

        fieldLayoutPub = table.getStringTopic("FieldLayout/Current").publish();
        fieldLayoutOptionsPub = table.getStringArrayTopic("FieldLayout/Options").publish();

        String[] layoutOptions = {WELDED_LAYOUT.toString(), ANDYMARK_LAYOUT.toString()};
        fieldLayoutOptionsPub.set(layoutOptions);
        fieldLayoutPub.set(WELDED_LAYOUT.toString());
        table.getStringTopic("FieldLayout/Selected").publish().set(WELDED_LAYOUT.toString());
        
        fieldLayoutListener = NetworkTableInstance.getDefault().addListener(
            table.getTopic("FieldLayout/Selected"), 
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
            event -> {
                if (event.valueData.equals(null)) {
                    String selectedLayout = event.valueData.value.getString();
                    setFieldLayout(selectedLayout);
                }
            }
        );
        
        initializeScorePositions();

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

    private void setFieldLayout(String layoutName) {
        if (layoutName.equals(currentFieldLayout)) {
            return;
        }
        
        AprilTagFieldLayout fieldLayout;
        try {
            if (layoutName.equals(ANDYMARK_LAYOUT.toString())) {
                fieldLayout = AprilTagFieldLayout.loadFromResource(ANDYMARK_LAYOUT.m_resourceFile);
                currentFieldLayout = ANDYMARK_LAYOUT.toString();
            } else {
                fieldLayout = AprilTagFieldLayout.loadFromResource(WELDED_LAYOUT.m_resourceFile);
                currentFieldLayout = WELDED_LAYOUT.toString();
            }
            
            photonPoseEstimatorLeft.setFieldTags(fieldLayout);
            photonPoseEstimatorRight.setFieldTags(fieldLayout);
            
            fieldLayoutPub.set(currentFieldLayout);
        } catch (Exception e) {
            e.printStackTrace();
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
                } else if(!secondCamResults.isEmpty() && secondCamResults.get(secondCamResults.size()-1).hasTargets()) {
                    // Only second camera sees
                    var secondRes = secondCamResults.get(secondCamResults.size()-1);
                    actualCenterX = getCenterX(secondRes.getBestTarget().detectedCorners);
                    desiredCenterX = targetPositions.get(secondCam).get(position);
                }

                offset = actualCenterX - desiredCenterX;
                targetCenterXPub.set(actualCenterX);
                desiredXPub.set(desiredCenterX);

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

    private boolean isAprilTagVisible() {
        return !leftCameraResults.isEmpty() && leftCameraResults.get(leftCameraResults.size() - 1).hasTargets() ||
            !rightCameraResults.isEmpty() && rightCameraResults.get(rightCameraResults.size() - 1).hasTargets();
    }

    public Command runOdometryAlign() {
        final double driveSpeed = 0.5;
        final double rotationSpeed = 0.5;
        selectedScorePosition = findClosestScorePosition();
        Pose2d targetPose = getScorePose(selectedScorePosition);
        // how off we can be
        final Distance positionTolerance = Meters.of(.05);
        final Angle rotationTolerance = Degrees.of(5);
        
        final DoublePublisher targetXPub = table.getDoubleTopic("Alignment/OdomTarget/X").publish();
        final DoublePublisher targetYPub = table.getDoubleTopic("Alignment/OdomTarget/Y").publish();
        final DoublePublisher targetRotPub = table.getDoubleTopic("Alignment/OdomTarget/Rotation").publish();
        
        return new RunCommand(
            () -> {
                Pose2d currentPose = getPose();
                // diff between current and target
                Distance xError = Meters.of(targetPose.getX() - currentPose.getX());
                Distance yError = Meters.of(targetPose.getY() - currentPose.getY());
                Angle rotationError = Degrees.of(targetPose.getRotation().minus(currentPose.getRotation()).getRadians()); //-pi to pi
                
                targetXPub.set(targetPose.getX());
                targetYPub.set(targetPose.getY());
                targetRotPub.set(targetPose.getRotation().getDegrees());
                
                Distance distance = Meters.of(Math.sqrt(Math.pow(xError.in(Meters), 2) + Math.pow(yError.in(Meters), 2)));
                
                if (distance.gt(positionTolerance)) {
                    double xOutput = xError.div(distance).times(driveSpeed).magnitude();
                    double yOutput = yError.div(distance).times(driveSpeed).magnitude();
                    double rotOutput = Math.signum(rotationError.in(Degrees)) *
                    (Math.abs(Math.abs(rotationError.in(Degrees)) > rotationTolerance.in(Degrees) ? rotationSpeed : 0));
                    
                    drive(
                        new Translation2d(xOutput, yOutput),
                        rotOutput,
                        true,
                        true
                    );

                } else if (Math.abs(rotationError.in(Degrees)) > rotationTolerance.in(Degrees)) {
                    drive(
                        new Translation2d(0, 0),
                        Math.signum(rotationError.in(Degrees)) * rotationSpeed,
                        true,
                        true
                    );

                } else { //if we're at the target
                    drive(new Translation2d(0, 0), 0, true, true);
                }
            }
        ).until(() -> {
            Pose2d currentPose = getPose();
            double positionError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotationError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
            
            return positionError < positionTolerance.in(Meters) && rotationError < rotationTolerance.in(Degrees);
        }).withTimeout(5); 
    }
    
    public enum ReefScorePosition {
        FRONT,
        FRONTLEFT,
        BACKLEFT,
        BACK,
        BACKRIGHT,
        FRONTRIGHT
    }
    
    private Map<ReefScorePosition, Pose2d> scorePositions = new HashMap<>();
    
    private void initializeScorePositions() {
        scorePositions.put(ReefScorePosition.FRONT, new Pose2d(1.84, 8.2, Rotation2d.fromDegrees(-90)));
        scorePositions.put(ReefScorePosition.FRONTLEFT, new Pose2d(-0.04, 5.55, Rotation2d.fromDegrees(0)));
        scorePositions.put(ReefScorePosition.BACKLEFT, new Pose2d(-0.04, 4.98, Rotation2d.fromDegrees(0)));
        scorePositions.put(ReefScorePosition.BACK, new Pose2d(0.36, 0.88, Rotation2d.fromDegrees(60)));
        scorePositions.put(ReefScorePosition.BACKRIGHT, new Pose2d(1.46, 0.25, Rotation2d.fromDegrees(60)));
        scorePositions.put(ReefScorePosition.FRONTRIGHT, new Pose2d(11.9, 3.71, Rotation2d.fromDegrees(-60)));
    }
            
    public ReefScorePosition findClosestScorePosition() {
        Pose2d currentPose = getPose();
        ReefScorePosition closestPosition = ReefScorePosition.FRONT;
        double closestDistance = Double.MAX_VALUE;
        
        for (Map.Entry<ReefScorePosition, Pose2d> entry : scorePositions.entrySet()) {
            double distance = currentPose.getTranslation().getDistance(entry.getValue().getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPosition = entry.getKey();
            }
        }
        
        return closestPosition;
    }

    public Command runTrajectoryOdomAlign() {
        // find closest score position to align with
        selectedScorePosition = findClosestScorePosition();
        Pose2d targetPose = getScorePose(selectedScorePosition);
        Pose2d currentPose = getPose();
        
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
                runOdometryAlign(),
                this::isAprilTagVisible
            )
        );
    }    

    public Pose2d getScorePose(ReefScorePosition position) {
        return scorePositions.get(position);
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
        selectedPositionPub.set(selectedScorePosition.toString());
    }

    
}