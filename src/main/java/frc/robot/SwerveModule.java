package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public final int moduleNumber;
    private final Rotation2d angleOffset;

    private final TalonFX angleMotor;
    private final StatusSignal<Angle> anglePosition;
    private final TalonFX driveMotor;
    private final StatusSignal<Angle> drivePosition;
    private final CANcoder angleEncoder;
    private final StatusSignal<Angle> angleEncoderPosition;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocityReq = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePositionReq = new PositionVoltage(0);

    private final DCMotor driveMotorSimConstants = DCMotor.getKrakenX60(1);
    private final DCMotor steerMotorSimConstants = DCMotor.getKrakenX60(1);
    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            driveMotorSimConstants, 
            0.025,
            Constants.Swerve.chosenModule.driveGearRatio),
        driveMotorSimConstants);
    private final DCMotorSim steerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            steerMotorSimConstants, 
            0.004, 
            Constants.Swerve.chosenModule.angleGearRatio), 
        driveMotorSimConstants);
    private final TalonFXSimState driveMotorSim;
    private final TalonFXSimState angleMotorSim;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);

        Robot.ctreConfigs.swerveDriveFXConfig.Slot0 = Slot0Configs.from(moduleConstants.driveGains);
        Robot.ctreConfigs.swerveAngleFXConfig.Slot0 = Slot0Configs.from(moduleConstants.angleGains);
        
        angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);

        drivePosition = driveMotor.getPosition();
        anglePosition = angleMotor.getPosition();
        angleEncoderPosition = angleEncoder.getAbsolutePosition();

        driveMotorSim = driveMotor.getSimState();
        angleMotorSim = angleMotor.getSimState();
        angleEncoder.getSimState().setRawPosition(angleOffset.getRotations());

        resetToAbsolute();
    }

    //made helper methods for sysid since module drive and angle motors aren't visible
    public void setDriveVoltage(double volts) {
        driveMotor.setControl(new VoltageOut(volts));
    }

    public void setSteerVoltage(double volts) {
        angleMotor.setControl(new VoltageOut(volts));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);
        angleMotor.setControl(anglePositionReq.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocityReq.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveMotor.setControl(driveVelocityReq);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoderPosition.getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        angleMotor.setPosition(absolutePosition);
    }

    /** Use this to obtain the drive position status signal of this module to be refreshed along with the signal from getAnglePosition every loop before using getState or getPosition */
    public BaseStatusSignal getDrivePosition() {
        return drivePosition;
    }

    /** Use this to obtain the angle position status signal of this module to be refreshed along with the signal from getDrivePosition every loop before using getState or getPosition */
    public BaseStatusSignal getAnglePosition() {
        return anglePosition;
    }

    /** Use this to obtain the encoder position status signal of this module to be refreshed every loop before using getCANcoder */
    public BaseStatusSignal getEncoderPosition() {
        return angleEncoderPosition;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(drivePosition.getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(anglePosition.getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(drivePosition.getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(anglePosition.getValueAsDouble())
        );
    }

    /** @return simulated current draw of the module in amps */
    public double simulationPeriodic() {
        var supplyVoltage = RobotController.getBatteryVoltage();
        driveMotorSim.setSupplyVoltage(supplyVoltage);
        angleMotorSim.setSupplyVoltage(supplyVoltage);
        driveSim.setInputVoltage(driveMotorSim.getMotorVoltage());
        steerSim.setInputVoltage(angleMotorSim.getMotorVoltage());
        driveSim.update(Constants.LOOP_TIME_SECONDS);
        steerSim.update(Constants.LOOP_TIME_SECONDS);

        driveMotorSim.setRawRotorPosition(driveSim.getAngularPositionRotations() * Constants.Swerve.chosenModule.driveGearRatio);
        driveMotorSim.setRotorVelocity(Units.radiansToRotations(driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.chosenModule.driveGearRatio));
        angleMotorSim.setRawRotorPosition(steerSim.getAngularPositionRotations() * Constants.Swerve.chosenModule.angleGearRatio);
        angleMotorSim.setRotorVelocity(Units.radiansToRotations(steerSim.getAngularVelocityRadPerSec()) * Constants.Swerve.chosenModule.angleGearRatio);

        return Math.abs(driveSim.getCurrentDrawAmps()) + Math.abs(steerSim.getCurrentDrawAmps());
    }
}