package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Robot;
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] mSwerveMods;
    private final BaseStatusSignal[] modStatusSignals;
    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSim;
    private final StatusSignal<Angle> gyroYaw;
    private final SysIdRoutine driveSysIdRoutine;
    private final SysIdRoutine steerSysIdRoutine;

    private final PhotonCamera leftCamera;
    private List<PhotonPipelineResult> leftCameraResults = List.of();
    private final PhotonCamera rightCamera;
    private List<PhotonPipelineResult> rightCameraResults = List.of();
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
    //private final HttpCamera camStream;

    private double simCurrentDrawAmps = 0;
    private final DoubleEntry xPosEntry;
    private long xPosEntryLastChanged;
    private final DoubleEntry yPosEntry;
    private long yPosEntryLastChanged;
    private final DoubleEntry rotEntry;
    private long rotEntryLastChanged;
    
    public Swerve() {
        field = new Field2d();
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        gyroSim = gyro.getSimState();
        gyroYaw = gyro.getYaw();
        alignmentPID = new PIDController(0.15, 0, 0); 
        alignmentPID.setTolerance(10, 10);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        modStatusSignals = new BaseStatusSignal[]{
            mSwerveMods[0].getDrivePosition(),
            mSwerveMods[0].getDriveVelocity(),
            mSwerveMods[0].getAnglePosition(),
            mSwerveMods[0].getEncoderPosition(),
            mSwerveMods[1].getDrivePosition(),
            mSwerveMods[1].getDriveVelocity(),
            mSwerveMods[1].getAnglePosition(),
            mSwerveMods[1].getEncoderPosition(),
            mSwerveMods[2].getDrivePosition(),
            mSwerveMods[2].getDriveVelocity(),
            mSwerveMods[2].getAnglePosition(),
            mSwerveMods[2].getEncoderPosition(),
            mSwerveMods[3].getDrivePosition(),
            mSwerveMods[3].getDriveVelocity(),
            mSwerveMods[3].getAnglePosition(),
            mSwerveMods[3].getEncoderPosition(),
            gyroYaw
        };

        //TODO: rename cameras
        leftCamera = new PhotonCamera(Constants.VisionConstants.leftCameraName);
        leftCameraAlert = new Alert(
            String.format("Left camera %s is not connected", Constants.VisionConstants.leftCameraName), 
            AlertType.kError);

        //camStream = new HttpCamera("Photonvison Left", "http://photonvision.local:1181");
        rightCamera = new PhotonCamera(Constants.VisionConstants.rightCameraName);
        rightCameraAlert = new Alert(
            String.format("Right camera %s is not connected", Constants.VisionConstants.rightCameraName), 
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
        if(Constants.IS_SIM) {
            Robot.registerFastPeriodic(() -> updateOdom());
        }
    
        final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
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
        if(Constants.IS_SIM) {
            xPosEntry = table.getDoubleTopic("Simulation/SetOdom/X").getEntry(0);
            xPosEntry.set(0);
            yPosEntry = table.getDoubleTopic("Simulation/SetOdom/Y").getEntry(0);
            yPosEntry.set(0);
            rotEntry = table.getDoubleTopic("Simulation/SetOdom/Rotation").getEntry(0);
            rotEntry.set(0);
        } else {
            xPosEntry = null;
            yPosEntry = null;
            rotEntry = null;
        }

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
        SmartDashboard.putData("Align Center", defer(() -> runTrajectoryOdomAlign(AlignmentPosition.CENTER)));
        SmartDashboard.putData("Align Left", defer(() -> runTrajectoryOdomAlign(AlignmentPosition.LEFT)));
        SmartDashboard.putData("Align Right", defer(() -> runTrajectoryOdomAlign(AlignmentPosition.RIGHT)));
        SmartDashboard.putData("Stop Drive", runOnce(() -> stop()));

        SmartDashboard.putData("DriveSysIdQuasiFwd", sysIdDriveQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("DriveSysIdQuasiRev", sysIdDriveQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("DriveSysIdDynFwd", sysIdDriveDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("DriveSysIdDynRev", sysIdDriveDynamic(SysIdRoutine.Direction.kReverse));
        
        SmartDashboard.putData("SteerSysIdQuasiFwd", sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SteerSysIdQuasiRev", sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SteerSysIdDynFwd", sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SteerSysIdDynRev", sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));
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

    private void stop() {
        drive(new Translation2d(), 0, false, false);
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
        return runOnce(() -> 
        {   var alliance = DriverStation.getAlliance();
            setPose(
                new Pose2d(
                    getPose().getTranslation(),
                    alliance.isPresent() 
                    && alliance.get() == DriverStation.Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d()
                )
            );
        });
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

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyroYaw.getValueAsDouble());
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
        ArrayList<Pose2d> scorePositionsList = new ArrayList<>();
        Map<Pose2d, ReefScorePosition> locations = new HashMap<>();
        
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
        // Red Alliance
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
    
        scorePositions = new ReefPositions(locations, scorePositionsList);
    }

    public Command runTrajectoryOdomAlign(AlignmentPosition position) {
        Pose2d currentPose = getPose();
        Pose2d targetPose = currentPose.nearest(scorePositions.poses);
        ReefScorePosition reefPosition = scorePositions.locations.get(targetPose);
        var alliance = DriverStation.getAlliance();
        boolean isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        double offsetDistance = .35; //meters
        double xOffset = 0.0;
        
        switch(position) {
            case LEFT:
                if (isRedAlliance)
                    xOffset = offsetDistance; //works for red alliance
                else 
                    xOffset = -offsetDistance; //works for blue alliance
                break;
            case RIGHT:
                if (isRedAlliance) 
                    xOffset = -offsetDistance;
                else 
                    xOffset = offsetDistance;
                break;
            case CENTER:
            default:
                xOffset = 0.0;
                break;
        }
        
        double offsetX, offsetY;
        
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
            default:
                offsetX = 0;
                offsetY = 0;
        }
        
        Pose2d offsetTargetPose = new Pose2d(
            targetPose.getX() + offsetX,
            targetPose.getY() + offsetY,
            targetPose.getRotation()
        );

        targetXPub.set(offsetTargetPose.getX());
        targetYPub.set(offsetTargetPose.getY());
        targetRotPub.set(offsetTargetPose.getRotation().getDegrees());

        SmartDashboard.putNumber("Original Target X", targetPose.getX());
        SmartDashboard.putNumber("Original Target Y", targetPose.getY());
        SmartDashboard.putNumber("Offset Target X", offsetTargetPose.getX());
        SmartDashboard.putNumber("Offset Target Y", offsetTargetPose.getY());
        SmartDashboard.putNumber("X Offset Applied", offsetX);
        SmartDashboard.putNumber("Y Offset Applied", offsetY);
    
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

        // resets odometry to the starting pose, follows trajectory, and stops robot
        return AutoBuilder.followPath(path);
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(modStatusSignals);
        for(SwerveModule mod : mSwerveMods){
            cancoderPubs[mod.moduleNumber].set(mod.getCANcoder().getDegrees());
            var modState = mod.getState();
            anglePubs[mod.moduleNumber].set(modState.angle.getDegrees());
            velocityPubs[mod.moduleNumber].set(modState.speedMetersPerSecond);
        }

        if(leftCamera.isConnected()) {
            leftCameraResults = leftCamera.getAllUnreadResults();
            leftCameraAlert.set(false);
        } else {
            leftCameraAlert.set(true);
        }
        if(rightCamera.isConnected()) {
            rightCameraResults = rightCamera.getAllUnreadResults();
            rightCameraAlert.set(false);
        } else {
            rightCameraAlert.set(true);
        }

        Pose2d currentPose = getPose();
        if(!Constants.IS_SIM) updateOdom();
        // TODO re-enable at least one camera after verifying no memory issues
        // if (!rightCameraResults.isEmpty()) {
        //     var photonRightUpdate = photonPoseEstimatorRight.update(rightCameraResults.get(rightCameraResults.size() - 1));
        //     if (photonRightUpdate.isPresent()) {
        //         EstimatedRobotPose visionPose = photonRightUpdate.get();
        //         poseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
        //     }
        // }
        
        if (!leftCameraResults.isEmpty()) {
            var photonLeftUpdate = photonPoseEstimatorLeft.update(leftCameraResults.get(leftCameraResults.size() - 1));
            if (photonLeftUpdate.isPresent()) {
                EstimatedRobotPose visionPose = photonLeftUpdate.get();
                poseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
            }
        }

        currentPose = getPose();
        field.setRobotPose(currentPose);

        alignmentPositionPub.set(currentAlignmentPosition.toString());
        isAlignedPub.set(isAligned(currentAlignmentPosition));
    }

    @Override
    public void simulationPeriodic() {
        simCurrentDrawAmps = 0;
        for(var mod : mSwerveMods) {
            simCurrentDrawAmps += mod.simulationPeriodic();
        }

        boolean resetRequested = false;
        var curPose = getPose();
        var x = curPose.getX();
        if(xPosEntry.getLastChange() != xPosEntryLastChanged) {
            resetRequested = true;
            xPosEntryLastChanged = xPosEntry.getLastChange();
            x = xPosEntry.get();
        }
        var y = curPose.getY();
        if(yPosEntry.getLastChange() != yPosEntryLastChanged) {
            resetRequested = true;
            yPosEntryLastChanged = yPosEntry.getLastChange();
            y = xPosEntry.get();
        }
        var rot = curPose.getRotation().getDegrees();
        if(rotEntry.getLastChange() != rotEntryLastChanged) {
            resetRequested = true;
            rotEntryLastChanged = rotEntry.getLastChange();
            rot = rotEntry.get();
        }
        if(resetRequested) {
            setPose(new Pose2d(x, y, Rotation2d.fromDegrees(rot)));
        } else {
            xPosEntry.set(x);
            xPosEntryLastChanged = xPosEntry.getLastChange();
            yPosEntry.set(y);
            yPosEntryLastChanged = yPosEntry.getLastChange();
            rotEntry.set(rot);
            rotEntryLastChanged = rotEntry.getLastChange();
        }
    }

    public double getSimulatedCurrentDrawAmps() {
        return simCurrentDrawAmps;
    }

    private void updateOdom() {
        if(Constants.IS_SIM) {
            gyroSim.setRawYaw(Units.radiansToDegrees(
                getGyroYaw().getRadians() + getRobotRelativeSpeeds().omegaRadiansPerSecond * Constants.LOOP_TIME_SECONDS));
        }
        poseEstimator.update(getGyroYaw(), getModulePositions());
    }
}