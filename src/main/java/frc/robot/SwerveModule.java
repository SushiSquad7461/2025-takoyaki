package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);

        switch(moduleNumber) {
            case 0 -> {
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod0.driveKS;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod0.driveKV;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod0.driveKA;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod0.angleKS;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod0.angleKV;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod0.angleKA;
            }
            case 1 -> {
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod1.driveKS;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod1.driveKV;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod1.driveKA;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod1.angleKS;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod1.angleKV;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod1.angleKA;

            }
            case 2 -> {
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod2.driveKS;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod2.driveKV;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod2.driveKA;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod2.angleKS;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod2.angleKV;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod2.angleKA;
            }
            case 3 -> {
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod3.driveKS;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod3.driveKV;
                Robot.ctreConfigs.swerveDriveFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod3.driveKA;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kS = Constants.DriveCharacterization.Mod3.angleKS;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kV = Constants.DriveCharacterization.Mod3.angleKV;
                Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kA = Constants.DriveCharacterization.Mod3.angleKA;
            }
        }
        
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

    }

    //made helper methods for sysid since module drive and angle motors aren't visible
    public void setDriveVoltage(double volts) {
        mDriveMotor.setControl(new VoltageOut(volts));
    }

    public void setSteerVoltage(double volts) {
        mAngleMotor.setControl(new VoltageOut(volts));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue().in(Units.Rotations))
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue().in(Units.Rotations))
        );
    }
}