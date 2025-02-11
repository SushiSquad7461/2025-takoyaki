package frc.util.control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.ClosedLoopConfig;

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

   private final NetworkTable pidTable;
   private final DoublePublisher kPPub;
   private final DoublePublisher kIPub;
   private final DoublePublisher kDPub;
   private final DoublePublisher kFPub;
   private final DoubleSubscriber kPSub;
   private final DoubleSubscriber kISub;
   private final DoubleSubscriber kDSub;
   private final DoubleSubscriber kFSub;


   public PIDConfig(double p, double i, double d, double f) {
      this.P = p;
      this.I = i;
      this.D = d;
      this.F = f;

      //network table implementation
      pidTable = NetworkTableInstance.getDefault().getTable("PIDTuning");
      
      kPPub = pidTable.getDoubleTopic("kP").publish();
      kIPub = pidTable.getDoubleTopic("kI").publish();
      kDPub = pidTable.getDoubleTopic("kD").publish();
      kFPub = pidTable.getDoubleTopic("kF").publish();
      
      kPSub = pidTable.getDoubleTopic("kP").subscribe(p);
      kISub = pidTable.getDoubleTopic("kI").subscribe(i);
      kDSub = pidTable.getDoubleTopic("kD").subscribe(d);
      kFSub = pidTable.getDoubleTopic("kF").subscribe(f);
      if (Constants.TUNING_MODE)
         publishValues();
   }

   public PIDConfig(double p, double d, double f) { this(p, 0.0, d, f); }

   public PIDConfig(double p, double f) { this(p, 0.0, 0.0, f); }

   public PIDConfig(double p) { this(p, 0.0, 0.0, 0.0); }

   public static PIDConfig getZeroPid() { return new PIDConfig(0.0, 0.0, 0.0, 0.0); }

   public static PIDConfig getPid(double p, double i, double d, double f) { return new PIDConfig(p, i, d, f); }

   public static PIDConfig getPid(double p, double d, double f) { return new PIDConfig(p, d, f); }

   public static PIDConfig getPid(double p, double f) { return new PIDConfig(p, f); }

   public static PIDConfig getPid(double p) { return new PIDConfig(p); }

   public void setPid(TalonFX talon) {
      Slot0Configs slot0Configs = new Slot0Configs();
      slot0Configs.kV = getF();
      slot0Configs.kP = getP();
      slot0Configs.kI = getI();
      slot0Configs.kD = getD();
      talon.getConfigurator().apply(slot0Configs);
   }

   public void updatePidConfig(TalonFXConfiguration config) {
      Slot0Configs slot0Configs = new Slot0Configs();
   
      slot0Configs.kV = getF();
      slot0Configs.kP = getP();
      slot0Configs.kI = getI();
      slot0Configs.kD = getD();
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

   private void publishValues() {
      kPPub.set(getP());
      kIPub.set(getI());
      kDPub.set(getD());
      kFPub.set(getF());
   }
  
   public ClosedLoopConfig createSparkMaxConfig(){
      return new ClosedLoopConfig().pidf(getP(), getI(), getD(), getF());
   }

   public PIDController getPIDController() {
      return new PIDController(getP(), getI(), getD());
   }
}
