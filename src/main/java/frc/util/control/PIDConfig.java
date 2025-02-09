package frc.util.control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class PIDConfig {
   //default pid values
   public final double P;
   public final double I;
   public final double D;
   public final double F;

   public final double G;
   public final double S;
   public final double V;
   public final double A;

   //pid
   private final NetworkTable pidTable;
   private final DoublePublisher kPPub;
   private final DoublePublisher kIPub;
   private final DoublePublisher kDPub;
   private final DoublePublisher kFPub;
   private final DoubleSubscriber kPSub;
   private final DoubleSubscriber kISub;
   private final DoubleSubscriber kDSub;
   private final DoubleSubscriber kFSub;

   //feedforward
   private final DoublePublisher kGPub;
   private final DoublePublisher kSPub;
   private final DoublePublisher kVPub;
   private final DoublePublisher kAPub;
   private final DoubleSubscriber kGSub;
   private final DoubleSubscriber kSSub;
   private final DoubleSubscriber kVSub;
   private final DoubleSubscriber kASub;


   public PIDConfig(double p, double i, double d, double f, double g, double s, double v, double a) {
      this.P = p;
      this.I = i;
      this.D = d;
      this.F = f;
      this.G = g;
      this.S = s;
      this.V = v;
      this.A = a;

      //network table implementation
      pidTable = NetworkTableInstance.getDefault().getTable("PIDTuning");
      
      kPPub = pidTable.getDoubleTopic("kP").publish();
      kIPub = pidTable.getDoubleTopic("kI").publish();
      kDPub = pidTable.getDoubleTopic("kD").publish();
      kFPub = pidTable.getDoubleTopic("kF").publish();
      kGPub = pidTable.getDoubleTopic("kG").publish();
      kSPub = pidTable.getDoubleTopic("kS").publish();
      kVPub = pidTable.getDoubleTopic("kV").publish();
      kAPub = pidTable.getDoubleTopic("kA").publish();

      kPSub = pidTable.getDoubleTopic("kP").subscribe(p);
      kISub = pidTable.getDoubleTopic("kI").subscribe(i);
      kDSub = pidTable.getDoubleTopic("kD").subscribe(d);
      kFSub = pidTable.getDoubleTopic("kF").subscribe(f);
      kGSub = pidTable.getDoubleTopic("kG").subscribe(g);
      kSSub = pidTable.getDoubleTopic("kS").subscribe(s);
      kVSub = pidTable.getDoubleTopic("kV").subscribe(v);
      kASub = pidTable.getDoubleTopic("kA").subscribe(a);

      if (Constants.TUNING_MODE)
         publishValues();
   }

   public PIDConfig(double p, double d, double f) { this(p, 0.0, d, f, 0.0, 0.0, 0.0, 0.0); }

   public PIDConfig(double p, double f) { this(p, 0.0, 0.0, f, 0.0, 0.0, 0.0, 0.0); }

   public PIDConfig(double p) { this(p, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

   public static PIDConfig getZeroPid() { return new PIDConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

   public static PIDConfig getPid(double p, double i, double d, double f, double g, double s, double v, double a) { return new PIDConfig(p, i, d, f, g, s, v, a); }

   public static PIDConfig getPid(double p, double i, double d, double f) { return new PIDConfig(p, i, d, f, 0.0, 0.0, 0.0, 0.0); }

   public static PIDConfig getPid(double p, double d, double f) { return new PIDConfig(p, d, f); }

   public static PIDConfig getPid(double p, double f) { return new PIDConfig(p, f); }

   public static PIDConfig getPid(double p) { return new PIDConfig(p); }

   public ArmFeedforward getArmFeedforward() {
      return new ArmFeedforward(getS(), getG(), getV(), getA());    
   }

   public void setPid(TalonFX talon) {
      Slot0Configs slot0Configs = new Slot0Configs();
      slot0Configs.kV = getF();
      slot0Configs.kP = getP();
      slot0Configs.kI = getI();
      slot0Configs.kD = getD();
      slot0Configs.kG = getG();
      slot0Configs.kS = getS();
      slot0Configs.kV = getV();
      slot0Configs.kA = getA();

      talon.getConfigurator().apply(slot0Configs);
   }

   public void updatePidConfig(TalonFXConfiguration config) {
      Slot0Configs slot0Configs = new Slot0Configs();
   
      slot0Configs.kV = getF();
      slot0Configs.kP = getP();
      slot0Configs.kI = getI();
      slot0Configs.kD = getD();
      slot0Configs.kG = getG();
      slot0Configs.kS = getS();
      slot0Configs.kV = getV();
      slot0Configs.kA = getA();

      config.withSlot0(slot0Configs);
   }

   public double getP() {
      return Constants.TUNING_MODE ? kPSub.get() : P;
   }

   public double getI() {
      return Constants.TUNING_MODE ? kISub.get() : I;
   }

   public double getD() {
      return Constants.TUNING_MODE ? kDSub.get() : D;
   }

   public double getF() {
      return Constants.TUNING_MODE ? kFSub.get() : F;
   }

   public double getG() {
      return Constants.TUNING_MODE ? kGSub.get() : G;
   }

   public double getS() {
      return Constants.TUNING_MODE ? kSSub.get() : S;
   }
   public double getV() {
      return Constants.TUNING_MODE ? kVSub.get() : V;
   }
   public double getA() {
      return Constants.TUNING_MODE ? kASub.get() : A;
   }


   private void publishValues() {
      kPPub.set(getP());
      kIPub.set(getI());
      kDPub.set(getD());
      kFPub.set(getF());
      kGPub.set(getG());
      kSPub.set(getS());
      kVPub.set(getV());
      kAPub.set(getA());

   }
  
   public ClosedLoopConfig createSparkMaxConfig(){
      return new ClosedLoopConfig().pidf(getP(), getI(), getD(), getF());
   }

   public PIDController getPIDController() {
      return new PIDController(getP(), getI(), getD());
   }
}
