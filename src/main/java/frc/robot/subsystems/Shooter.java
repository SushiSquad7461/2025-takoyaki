package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import edu.wpi.first.units.measure.Angle;


import com.ctre.phoenix6.controls.PositionDutyCycle;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;




public class Shooter extends SubsystemBase{
    private TalonFX pivotMotor;
    private TalonFX wheelMotor;

    PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public Shooter(){
        //TODO: Add the port numbers for both motors
        pivotMotor = new TalonFX(0);
        wheelMotor = new TalonFX(1);

        //Applies a specific config to each motor
        pivotMotor.getConfigurator().apply(Constants.Shooter.PIVOT_CONFIG);
        wheelMotor.getConfigurator().apply(Constants.Shooter.WHEEL_CONFIG);
    }


    private Command intakeAlgae(){
        return runOnce(() -> wheelMotor.set(Constants.Shooter.ROLLER_SPEED));
    }

    private Command shootAlgae(){
        return runOnce(()-> wheelMotor.set(-1*Constants.Shooter.ROLLER_SPEED));
    }

    private Command idleWheels(){
        return runOnce(()-> wheelMotor.set(0.0));
    }

    private Command changePivotPos(Angle pos){
        //This will use a Position Duty Cycle until intake reaches within a specific error range of the final position
        return runOnce( () -> pivotMotor.setControl(positionDutyCycle.withPosition(pos)))
        .andThen(Commands.waitUntil(rightPos(pos)));

    }
    private BooleanSupplier rightPos(Angle pos){
        /*This is a boolean value that will check to see whether position that the motor is
         * (when taking the abs value), within a specific range of the desired position.
         * This will allow for a small error margin
        */
        return () -> (pivotMotor.getPosition().getValue())
        .minus(pos).abs(Degrees) < Constants.Shooter.MAX_ERROR.in(Degrees);
    }

    public Command changeState(ShooterState state){
        //These states will determine what state each motor should be in
        Command pivotState;
        Command wheelState;
        if(state.extended){
            pivotState = changePivotPos(Constants.Shooter.LOWERED_POS);
        }
        else{
            if(state.shootPos == Constants.Shooter.BARGE_POS){
                pivotState=changePivotPos(Constants.Shooter.BARGE_POS);
            }
            else{
                pivotState = changePivotPos(Constants.Shooter.PROCESSOR_POS);
            }
        }
        //Use a switch-case to determine what the wheels should do at each direction state
        wheelState = switch(state.direction){
            case OFF -> idleWheels();
            case REVERSE -> shootAlgae();
            case FORWARD -> intakeAlgae();
        };

        return pivotState.andThen(wheelState);
    }

}