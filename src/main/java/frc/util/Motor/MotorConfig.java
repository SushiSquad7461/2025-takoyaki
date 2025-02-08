package frc.util.Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.util.Control.PIDConfig;
import frc.util.Control.SmartDashboard.PIDTuning;



public class MotorConfig{
   public final PIDConfig pid;
   public final int canId;
   public final String canBus;
   public final int currentLimit;
   public final boolean inversion;
   public final Mode mode;

   private DigitalInput beamBreakSensor;
@SuppressWarnings("unused")
private final int beamBreakPort;
@SuppressWarnings("unused")
   private ArmFeedforward wristFeedforward = new ArmFeedforward(0.0, 0.0, 0.0); // Needs tuning

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

   public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) {
        this.canId = canId;
        this.canBus = canBus;
        this.currentLimit = currentLimit;
        this.inversion = inversion;
        this.pid = pid;
        this.mode = mode;
        this.beamBreakPort = -1; 
   }

   public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode, int beamBreakPort){
        this(canId, canBus, currentLimit, inversion, pid, mode);
        this.beamBreakSensor = new DigitalInput(beamBreakPort);
   }
   public MotorConfig(int canId, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(canId, "rio", currentLimit, inversion, pid, mode); }

   public MotorConfig(int canId) { this(canId, "rio", -1, false, PIDConfig.getZeroPid(), Mode.COAST);}

   public MotorConfig(int canId, String canBus) { this(canId, canBus, -1, false, PIDConfig.getZeroPid(), Mode.COAST);}

   public MotorConfig(int canId, int currentLimit, Boolean inversion, Mode mode) { this(canId, "rio", currentLimit, inversion, PIDConfig.getZeroPid(), mode);}

   public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, Mode mode) { this(canId, canBus, currentLimit, inversion, PIDConfig.getZeroPid(), mode);}

   public PIDTuning genPIDTuning(String motorName, boolean tuningMode) {
        return new PIDTuning(motorName, pid, tuningMode);
   }

   public boolean isBeamBreakTriggered() {
        return beamBreakSensor != null && !beamBreakSensor.get(); 
   }

   public void setWristFeedforward(double ks, double kg, double kv) {
        this.wristFeedforward = new ArmFeedforward(ks, kg, kv);
   }

   public TalonFXConfiguration getTalonConfig() {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        MotorHelper.updateSupplyCurrentLimit(currentLimit, talonConfig);
 
        talonConfig.MotorOutput.NeutralMode = mode.getTalonMode();
        talonConfig.MotorOutput.Inverted = inversion ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        pid.updatePidConfig(talonConfig);

        // Soft Limits for Safety
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100; // Set based on  max position
        talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // Set based on  min position

        return talonConfig;
   }

   public void setCanSparkMaxConfig(SparkMax motor, MotorType type) {
     SparkMaxConfig config = new SparkMaxConfig();
     config.inverted(inversion);
     config.smartCurrentLimit(currentLimit);
     config.idleMode(mode.getSparkMaxMode());
     config.closedLoop.apply(pid.createSparkMaxConfig());
   }

   public TalonFX createTalon() {
        TalonFX motor = new TalonFX(canId, canBus);
        motor.getConfigurator().apply(getTalonConfig());
        return motor;
   }

   public SparkMax createSparkMax() { return createSparkMax(MotorType.kBrushless); }

   public SparkMax createSparkMax(MotorType type) {
        SparkMax motor = new SparkMax(canId, type);
        setCanSparkMaxConfig(motor, type);
        return motor;
   }

   public MotorConfig withCanId(int canId) {return new MotorConfig(canId, this.canBus, this.currentLimit, this.inversion, this.pid, this.mode);}
}