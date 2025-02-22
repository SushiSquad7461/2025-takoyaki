package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import frc.robot.util.control.PIDConfig;
import frc.robot.util.motor.MotorConfig;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
    public static boolean TUNING_MODE;
    public static final double stickDeadband = 0.1;

    public static class Ports {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int PROG_PORT = 2;

        public static final int ALGAE_INTAKE_ROLLER_ID = 15;
        public static final int INTAKE_PIVOT_ID = 17;
        public static final int ROLLER_MOTOR_ID = 16;

        public static final int ELEVATOR_LEFT_ID = 14;
        public static final int ELEVATOR_RIGHT_ID = 18;

        public static final int LIMIT_SWITCH_PORT = 1; 
        public static final int BEAM_BREAK_PORT = 2; 
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
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
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
        public static final double driveKP = 0.12;
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(169.628906);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(168.925781); //168.925781
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-15.732422);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-144.228516);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static class Elevator {
        public static final Distance MAX_HEIGHT = Inches.of(28.0); 
        public static final Distance ELEVATOR_EXTENSION_PER_ROTATION = Meters.of(0.1); //TODO: measure elevator height and divide by gear ratio (repeat 5x)
        public static final Dimensionless GEAR_RATIO = Rotations.of(52*60).div(Rotations.of(18*18)); //output over input
        public static final Angle MOTOR_MAX_HEIGHT = frc.robot.subsystems.Elevator.heightToMotor(MAX_HEIGHT);

        // for motion magic, TODO: set and add jerk to motor config
        public static final AngularVelocity MOTION_MAGIC_VELOCITY = RotationsPerSecond.of(80);
        public static final AngularAcceleration MOTION_MAGIC_ACCELERATION = RotationsPerSecondPerSecond.of(160);
        public static final Per<AngularAccelerationUnit, TimeUnit> MOTION_MAGIC_JERK = Per.ofBaseUnits(0.0, PerUnit.combine(RotationsPerSecondPerSecond, Seconds)); //add towards end of tuning
        
        //TODO: use sysid and set all of these values
        public static final MotorConfig ELEVATOR_LEFT = new MotorConfig(
            Ports.ELEVATOR_LEFT_ID,
            35,
            true,
            PIDConfig.getElevatorPid(0.0, 0.0, 0, 0, 0, 0, 0),
            MotorConfig.Mode.BRAKE, 
            MOTOR_MAX_HEIGHT, 
            Degrees.of(0)).withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION);

        public static final MotorConfig ELEVATOR_RIGHT = new MotorConfig(
            Ports.ELEVATOR_RIGHT_ID,
            35,
            false,
            PIDConfig.getElevatorPid(0.0, 0.0, 0, 0, 0, 0, 0),
            MotorConfig.Mode.BRAKE,
            MOTOR_MAX_HEIGHT, 
            Degrees.of(0)).withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION);

        public static final Angle MAX_ERROR = Degrees.of(1.0);
    }

    public static final class CoralManipulator {
        // motion and position control w/ pivot
        public static final Angle MAX_ANGLE = Degrees.of(199.5);
        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle ANGLE_TOLERANCE = Degrees.of(5.0);
                        
        public static final MotorConfig ROLLER_CONFIG = new MotorConfig(
            Ports.ROLLER_MOTOR_ID,
            35,
            false,
            MotorConfig.Mode.BRAKE
        );
        
        // roller speeds for diff states (should be in range [-1, 1])
        public static final double INTAKE_SPEED = 0.5;
        public static final double SCORE_SPEED = 0.8;
        public static final double HOLD_SPEED = 0;
    }
  
    public static class AlgaeIntake {
        public static final Dimensionless INTAKE_GEAR_RATIO = Rotations.of(15).div(Rotations.of(1)); // output over input
        public static final double INTAKE_SPEED = 0.9;

        public static final Angle MAX_ERROR = Degrees.of(5.0);
        public static final Angle RAISED_POS = Degrees.of(199.5).div(INTAKE_GEAR_RATIO);
        public static final Angle LOWERED_POS = Degrees.of(0);

        public static final Current CURRENT_LIMIT = Amps.of(35.0);

        //TODO: use sysid and set all of these values
        public static final MotorConfig INTAKE_CONFIG = new MotorConfig(
            Ports.ALGAE_INTAKE_ROLLER_ID,
            0,
            true, 
            MotorConfig.Mode.COAST
        );

        public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
            Ports.INTAKE_PIVOT_ID,
            0,
            true,
            PIDConfig.getArmPid(0.0, 0.0, 0.0, 0, 0, 0, 0),
            MotorConfig.Mode.BRAKE
        );
    }
    
    public static final class AutoConstants { //TODO: Need to tune constants
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPTranslationController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
