package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;


public class Shooter extends SubsystemBase{
    private TalonFX pivotMotor;
    private TalonFX wheelMotor;
    PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public Shooter(){
        //TODO: Add the port numbers for both motors
        pivotMotor = new TalonFX(0);
        pivotMotor.getConfigurator().apply(Constants.Shooter.PIVOT_CONFIG);
        wheelMotor = new TalonFX(1);
        wheelMotor.getConfigurator().apply(Constants.Shooter.WHEEL_CONFIG);
    }


    public Command intakeAlgae(){
        return runOnce(() -> wheelMotor.set(Constants.Shooter.ROLLER_SPEED));
    }

    public Command shootAlgae(){
        return runOnce(()-> wheelMotor.set(-1*Constants.Shooter.ROLLER_SPEED));
    }

    public Command changePivotPos(Angle pos){
        return runOnce( () -> pivotMotor.setControl(positionDutyCycle.withPosition(pos))).andThen(Commands.waitUntil(intakeInPos(pos)));

    }
    public BooleanSupplier intakeInPos(Angle pos){
        return null;
    }

}