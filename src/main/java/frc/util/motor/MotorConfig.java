package frc.util.motor;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.util.control.PIDConfig;
import frc.util.control.nt.PIDTuning;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class MotorConfig {
   public final PIDConfig pid;
   public final int canId;
   public final String canBus;
   public final int currentLimit; // A currentLimit of bellow zero will not be set on the motor
   public final boolean inversion;
   public final Mode mode;

   private final AngularVelocity velocityLimit;
   private final AngularAcceleration accelerationLimit;

   private final Angle forwardSoftLimit;
   private final Angle reverseSoftLimit;

   public enum Mode {
        COAST(true),
        BRAKE(false);

        private boolean mode;

        private Mode(boolean mode) {
            this.mode = mode;
        }

        public NeutralModeValue getTalonMode() {
            return mode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        }

        public IdleMode getSparkMaxMode() {
            return mode ? IdleMode.kCoast : IdleMode.kBrake;
        }
   };

   public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode,
   AngularVelocity velocityLimit, AngularAcceleration accelerationLimit, Angle forwardSoftLimit, Angle reverseSoftLimit) {
        this.canId = canId;
        this.canBus = canBus;
        this.currentLimit = currentLimit;
        this.inversion = inversion;
        this.pid = pid;
        this.mode = mode;
        this.velocityLimit = velocityLimit;
        this.accelerationLimit = accelerationLimit;
        this.forwardSoftLimit = forwardSoftLimit;
        this.reverseSoftLimit = reverseSoftLimit;
   }

   public MotorConfig(int canId, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode, Angle forwardSoftLimit, Angle reverseSoftLimit) { this(canId, "rio", currentLimit, inversion, pid, mode, RadiansPerSecond.zero(), RadiansPerSecondPerSecond.zero(), forwardSoftLimit, reverseSoftLimit); }
   public MotorConfig(int canId, int currentLimit, Boolean inversion, Mode mode) { this(canId, "rio", currentLimit, inversion, PIDConfig.getZeroPid(), mode, RadiansPerSecond.zero(), RadiansPerSecondPerSecond.zero(), null, null); }

   public PIDTuning genPIDTuning(String motorName, boolean tuningMode) {
        return new PIDTuning(motorName, pid, tuningMode);
   }

   public TalonFXConfiguration getTalonConfig() {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        MotorHelper.updateSupplyCurrentLimit(currentLimit, talonConfig);
        if (forwardSoftLimit != null){
          talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit.in(Rotations);
        }

        if (reverseSoftLimit != null) {
          talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit.in(Rotations);  
        }
        

        talonConfig.MotorOutput.NeutralMode = mode.getTalonMode();
        talonConfig.MotorOutput.Inverted = inversion ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        pid.updatePidConfig(talonConfig);

        return talonConfig;
   }

   public TalonFX createTalon() {
        TalonFX motor = new TalonFX(canId, canBus);
        motor.getConfigurator().apply(getTalonConfig());
        return motor;
   }

   public void setCanSparkMaxConfig(SparkMax motor, MotorType type) {
     SparkMaxConfig config = new SparkMaxConfig();
     config.inverted(inversion);
     config.smartCurrentLimit(currentLimit);
     config.idleMode(mode.getSparkMaxMode());
     config.closedLoop.apply(pid.createSparkMaxConfig());

     motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     if (type == MotorType.kBrushless) {
          motor.getEncoder().setPosition(0);
     }

   }

   public SparkMax createSparkMax() { return createSparkMax(MotorType.kBrushless); }

   public SparkMax createSparkMax(MotorType type) {
        SparkMax motor = new SparkMax(canId, type);
        setCanSparkMaxConfig(motor, type);
        return motor;
   }

   public MotorConfig withCanId(int canId) {
     return new MotorConfig(canId, this.canBus, this.currentLimit, this.inversion, this.pid, this.mode, this.velocityLimit, this.accelerationLimit, this.forwardSoftLimit, this.reverseSoftLimit);
   }
   
   public TalonFX createTalonForMotionMagic() {
     TalonFX motor = new TalonFX(canId, canBus);
     TalonFXConfiguration config = getTalonConfig();
     
     MotionMagicConfigs motionMagic = new MotionMagicConfigs();
     motionMagic.MotionMagicCruiseVelocity = velocityLimit.in(RadiansPerSecond);
     motionMagic.MotionMagicAcceleration = accelerationLimit.in(RadiansPerSecondPerSecond);
     
     config.MotionMagic = motionMagic;
     motor.getConfigurator().apply(config);
     return motor;
   }
   
   public MotorConfig withMotionMagic(
     AngularVelocity velocityLimit,
     AngularAcceleration accelerationLimit) {
     return new MotorConfig(
          this.canId, this.canBus, this.currentLimit, this.inversion, 
          this.pid, this.mode, velocityLimit, accelerationLimit, forwardSoftLimit, reverseSoftLimit);
     }
}
