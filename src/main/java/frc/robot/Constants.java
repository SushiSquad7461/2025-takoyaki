package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Swerve.AlignmentPosition;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
    public static final boolean IS_SIM = Robot.isSimulation();
    public static final double stickDeadband = 0.1;
    public static final double LOOP_TIME_SECONDS = 0.02;
    public static final double FAST_LOOP_TIME_SECONDS = 0.01;

    public static final CurrentLimitsConfigs BASIC_CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimit(35);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CW = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CCW = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    public static class Ports {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int PROG_PORT = 2;

        public static final int ALGAE_INTAKE_ROLLER_ID = 15;
        public static final int INTAKE_PIVOT_ID = 17;
        public static final int CORAL_ROLLER_MOTOR_ID = 16;

        public static final int ELEVATOR_LEFT_ID = 14;
        public static final int ELEVATOR_RIGHT_ID = 18;

        public static final int LIMIT_SWITCH_PORT = 2; 
        public static final int BEAM_BREAK_PORT = 3;
        public static final int ELEV_BEAM_BREAK_PORT = 6; 
    }

    public static class CustomUnits {
        public static final PerUnit<DistanceUnit, AngleUnit> MetersPerRotation = Meters.per(Rotations);
    }

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean REDUCE_SPEED = true;
        public static final double LOW_SPEED = 0.1;
        public static final double LOW_ROT = 0.1;
        public static final int CAMERA_RESOLUTIONX = 1280;
        
        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), //front left => Mod 0
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), //front right => Mod 1
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //back left => Mod 2
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); //back right => Mod 3

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 40;
        public static final int angleCurrentLowerLimit = 25;
        public static final double angleCurrentLowerTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentLowerLimit = 35;
        public static final double driveCurrentLowerTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.4;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(170.156250); //point bevel to right
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID, 
                angleMotorID, 
                canCoderID, 
                angleOffset,
                new SlotConfigs()
                    .withKP(driveKP)
                    .withKI(driveKI)
                    .withKD(driveKD)
                    .withKS(IS_SIM ? 0.010945 : 0.19)
                    .withKV(IS_SIM ? 0.72409  : 0.71)
                    .withKA(IS_SIM ? 0.044977  : 0.066),
                new SlotConfigs()
                    .withKP(angleKP)
                    .withKI(angleKI)
                    .withKD(angleKD)
                    .withKS(IS_SIM ? 0 :    0.13995)
                    .withKV(IS_SIM ? 0 :    0.2178)
                    .withKA(IS_SIM ? 0 :    0.14872));
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(132.802734);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID, 
                angleMotorID, 
                canCoderID, 
                angleOffset,
                new SlotConfigs()
                    .withKP(driveKP)
                    .withKI(driveKI)
                    .withKD(driveKD)
                    .withKS(IS_SIM ? 0.010945 : 0.18879)
                    .withKV(IS_SIM ? 0.72409  : 0.70816)
                    .withKA(IS_SIM ? 0.044977  : 0.066208),
                new SlotConfigs()
                    .withKP(angleKP)
                    .withKI(angleKI)
                    .withKD(angleKD)
                    .withKS(IS_SIM ? 0 :    0.16468)
                    .withKV(IS_SIM ? 0 :    2.2002)
                    .withKA(IS_SIM ? 0 :    0.12755));
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-16.259766);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID, 
                angleMotorID, 
                canCoderID, 
                angleOffset,
                new SlotConfigs()
                    .withKP(driveKP)
                    .withKI(driveKI)
                    .withKD(driveKD)
                    .withKS(IS_SIM ? 0.010945 : 0.1812)
                    .withKV(IS_SIM ? 0.72409  : 0.68313)
                    .withKA(IS_SIM ? 0.044977  : 0.031761),
                new SlotConfigs()
                    .withKP(angleKP)
                    .withKI(angleKI)
                    .withKD(angleKD)
                    .withKS(IS_SIM ? 0 :    0.1437)
                    .withKV(IS_SIM ? 0 :    2.1619)
                    .withKA(IS_SIM ? 0 :    0.037237));
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(30.585937);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID, 
                angleMotorID, 
                canCoderID, 
                angleOffset,
                new SlotConfigs()
                    .withKP(driveKP)
                    .withKI(driveKI)
                    .withKD(driveKD)
                    .withKS(IS_SIM ? 0.010945 : 0.15286)
                    .withKV(IS_SIM ? 0.72409  : 1.72792)
                    .withKA(IS_SIM ? 0.044977  : 0.059573),
                new SlotConfigs()
                    .withKP(angleKP)
                    .withKI(angleKI)
                    .withKD(angleKD)
                    .withKS(IS_SIM ? 0 :    0.18268)
                    .withKV(IS_SIM ? 0 :    2.2717)
                    .withKA(IS_SIM ? 0 :    0.046578));
        }
    }

    public static class Elevator {
        public static final Distance MAX_HEIGHT = Inches.of(28.0); 
        public static final Distance ELEVATOR_EXTENSION_PER_ROTATION = Inches.of(3.994); 
        public static final Double GEAR_RATIO = 52.*60./18./18.; //output over input
        public static final Angle MOTOR_MAX_HEIGHT = frc.robot.subsystems.Elevator.heightToMotor(MAX_HEIGHT);

        public static final AngularVelocity MOTION_MAGIC_VELOCITY = RotationsPerSecond.of(80);
        public static final AngularAcceleration MOTION_MAGIC_ACCELERATION = RotationsPerSecondPerSecond.of(160);
        public static final double CURRENT_LIMIT_AMPS = Constants.IS_SIM ? 4 : 5;

        public static final TalonFXConfiguration ELEVATOR_LEFT = new TalonFXConfiguration()
            .withMotorOutput(MOTOR_OUTPUT_CW);

        public static final TalonFXConfiguration ELEVATOR_RIGHT = new TalonFXConfiguration()
            .withMotorOutput(MOTOR_OUTPUT_CCW)
            .withSlot0(new Slot0Configs()
                .withKP(IS_SIM ? 0.14274      : 0.0055095)
                .withKD(IS_SIM ? 0            : 0.02)
                .withKG(IS_SIM ? 0.1494140625 : 0.17409)
                .withKS(IS_SIM ? 0.001953125  : 0.065134)
                .withKV(IS_SIM ? 0.146        : 0.11334)
                .withKA(IS_SIM ? 0.00475      : 0.0014578)
            ).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(MOTION_MAGIC_VELOCITY)
                .withMotionMagicAcceleration(MOTION_MAGIC_ACCELERATION)
            );
        
        public static final double MAX_ERROR_ROTATIONS = frc.robot.subsystems.Elevator.heightToMotor(Inches.of(.5)).in(Rotations);
        public static final double RELAXED_MAX_ERROR_ROTATIONS = frc.robot.subsystems.Elevator.heightToMotor(Inches.of(2.0)).in(Rotations);
        public static final double maxSimHeightMeters = Inches.of(28.0).in(Meters);
    }

    public static final class CoralManipulator {
        // motion and position control w/ pivot
        public static final TalonFXConfiguration ROLLER_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(BASIC_CURRENT_LIMIT)
            .withMotorOutput(MOTOR_OUTPUT_CW);
    }
  
    public static class AlgaeIntake {
        public static final double INTAKE_GEAR_RATIO = 15; // output over input
        public static final double INTAKE_SPEED = 0.4;

        public static final Angle MAX_ERROR = Degrees.of(5.0 * INTAKE_GEAR_RATIO);
        public static final Angle RAISED_POS = Degrees.of(12 * INTAKE_GEAR_RATIO);
        public static final Angle LOWERED_POS = Degrees.of(53 * INTAKE_GEAR_RATIO);
        public static final Angle INTAKE_ANGLE = Degrees.of(56.85 * INTAKE_GEAR_RATIO);

        public static final double CURRENT_SPIKE_LIMIT_DOWN_AMPS = 5;
        public static final double CURRENT_SPIKE_LIMIT_UP_AMPS = 5;

        public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(BASIC_CURRENT_LIMIT)
            .withMotorOutput(MOTOR_OUTPUT_CCW);

        public static final TalonFXConfiguration PIVOT_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(BASIC_CURRENT_LIMIT)
            .withMotorOutput(MOTOR_OUTPUT_CCW)
            .withSlot0(new Slot0Configs().withKP(0.2));
    }
    
    public static final class AutoConstants { //TODO: Need to tune constants!
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPTranslationController = IS_SIM ? 15 : 1;
        public static final double kPThetaController = 1;
    }

    public static final class VisionConstants { //TODO: only tell pipeline to give pose when multiple tags detected
        public static final Transform3d leftCamera = new Transform3d(new Translation3d(0.220726, 0.27351736, 0.1913255), new Rotation3d(0, -0.30543261909, -0.59));
        public static final Transform2d leftCamera2d = new Transform2d(new Translation2d(leftCamera.getX(), leftCamera.getY()), new Rotation2d(leftCamera.getRotation().getZ()));

        public static final Transform3d rightCamera = new Transform3d(new Translation3d(0.220726, -0.27351736, 0.1913255), new Rotation3d(0, -0.30543261909, 0.59));
        public static final Transform2d rightCamera2d = new Transform2d(new Translation2d(rightCamera.getX(), rightCamera.getY()), new Rotation2d(rightCamera.getRotation().getZ()));

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

        public static final String leftCameraName = "Arducam_OV9782_USB_Camera";
        public static final Map<AlignmentPosition, Double> leftCameraOffsets = Map.of( 
            AlignmentPosition.LEFT, 1107.,
            AlignmentPosition.RIGHT, -(double)Swerve.CAMERA_RESOLUTIONX, // TODO verify target offscreen in this case
            AlignmentPosition.CENTER, 839.
        );

        public static final String rightCameraName = "Arducam_OV9281_USB_Camera";
        public static final Map<AlignmentPosition, Double> rightCameraOffsets = Map.of(
            AlignmentPosition.LEFT, 2.*Swerve.CAMERA_RESOLUTIONX, // TODO verify target offscreen in this case
            AlignmentPosition.RIGHT, 308., //318
            AlignmentPosition.CENTER, 571. //.29
        );
    }

    public static final class AprilTags {
        public static final double[] coralBlueTags = { 17, 18, 19, 20, 21, 22 };
        public static final double[] coralRedTags =  { 6, 7, 8, 9, 10, 11 };

        public static final double[] stationBlueTags = { 12, 13 };
        public static final double[] stationRedTags =  { 1, 2 };

        // X is horizontal distance, Y is distance out from coral aprilTag -- half of bot width (bumpers included)
        public static final Translation2d coralOffsetRight = new Translation2d(0.147, 0.56); // 0.148
        public static final Translation2d coralOffsetLeft = new Translation2d(-0.147, 0.56);
        public static final Translation2d coralOffsetRightLow = new Translation2d(0.147, 0.62);
        public static final Translation2d coralOffsetLeftLow = new Translation2d(-0.147, 0.62);
        
        public static final Translation2d coralOffsetCentered = new Translation2d(-0.04, 0.46);

        public static final double coralXTagOffset = -0.04;

        public static final Translation2d coralCollectOffset = new Translation2d(0, 0.515); // 0.035

        public static final PathConstraints aprilTagDriveConstraints = new PathConstraints(0.4, 0.4, 0.4, 0.4);

        public static Map<Double, Double> aprilTagFaceAngles = new HashMap<Double, Double>();

        static {
        // blue side apriltags 
        aprilTagFaceAngles.put(Double.valueOf(17), Double.valueOf(60));
        aprilTagFaceAngles.put(Double.valueOf(18), Double.valueOf(0));
        aprilTagFaceAngles.put(Double.valueOf(19), Double.valueOf(-60));
        aprilTagFaceAngles.put(Double.valueOf(20), Double.valueOf(-120));
        aprilTagFaceAngles.put(Double.valueOf(21), Double.valueOf(180));
        aprilTagFaceAngles.put(Double.valueOf(22), Double.valueOf(120));

        // red side apriltags
        aprilTagFaceAngles.put(Double.valueOf(11), Double.valueOf(60));
        aprilTagFaceAngles.put(Double.valueOf(10), Double.valueOf(0));
        aprilTagFaceAngles.put(Double.valueOf(9), Double.valueOf(-60));
        aprilTagFaceAngles.put(Double.valueOf(8), Double.valueOf(-120));
        aprilTagFaceAngles.put(Double.valueOf(7), Double.valueOf(180));
        aprilTagFaceAngles.put(Double.valueOf(6), Double.valueOf(120));

        // coral intake apriltags
        aprilTagFaceAngles.put(Double.valueOf(13), Double.valueOf(125));
        aprilTagFaceAngles.put(Double.valueOf(12), Double.valueOf(-125));
        aprilTagFaceAngles.put(Double.valueOf(2), Double.valueOf(55));
        aprilTagFaceAngles.put(Double.valueOf(1), Double.valueOf(-55));
        }
  }

}

