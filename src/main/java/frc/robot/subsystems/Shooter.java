package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;




public class Shooter extends SubsystemBase{
    private TalonFX pivotMotor;
    private TalonFX wheelMotor;
    private TalonFX kickerMotor;

    //Creates a Position Duty Cycle
    PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public Shooter(){
        //TODO: Add the port numbers for both motors
        pivotMotor = new TalonFX(0);
        wheelMotor = new TalonFX(1);
        kickerMotor = new TalonFX(3);

        //Applies a specific config to each motor
        pivotMotor.getConfigurator().apply(Constants.AlgaeShooter.MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(Constants.AlgaeShooter.MOTOR_CONFIG);
        kickerMotor.getConfigurator().apply(Constants.AlgaeShooter.MOTOR_CONFIG);
    }

    //Will suck the algae in
    private Command intakeAlgae(){
        return runOnce(() -> wheelMotor.set(Constants.AlgaeShooter.ROLLER_SPEED));
    }

    //Will run the wheels
    private Command runWheels(){
        return runOnce(()-> 
        wheelMotor.set(-1*Constants.AlgaeShooter.ROLLER_SPEED));
    }

    //Will run the kicker
    private Command runKicker(){
        return runOnce(() -> kickerMotor.set(-1*Constants.AlgaeShooter.KICKER_SPEED));
    }

    //Will run the kicker and the wheels at the same time to shoot the algae
    private Command shootAlgae(){
        return runOnce(()-> runKicker().alongWith(runWheels()));
    }

    //Will make the wheels stop spinning
    private Command idleWheels(){
        return runOnce(()-> wheelMotor.set(0.0));
    }

    //I just used this to reset the intake, maybe use the current spike method later
    private Command reset(){
        return runOnce(()->
        idleWheels().andThen(changePivotPos(Constants.AlgaeShooter.UPRIGHT)));
    }

    //Changes the position of the pivot motor
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
        .minus(pos).abs(Degrees) < Constants.AlgaeShooter.MAX_ERROR.in(Degrees);
    }

    public Command changeState(ShooterState state){
        //These states will determine what state each motor should be in
        Command pivotState, wheelState;

        //Will cjeck to see if the intake is requesting to be intaking, if so, it will move all the way dowm
        if(state.intaking){
            pivotState = changePivotPos(Constants.AlgaeShooter.LOWERED_POS);
        }
        else{
            //This will check to see if the statw doesnt require the intake to be down
            if(state.shootPos == Constants.AlgaeShooter.BARGE_POS){
                pivotState = changePivotPos(Constants.AlgaeShooter.BARGE_POS)
                .andThen(Commands.waitSeconds(Constants.AlgaeShooter.REV_UP_TIME))
                .andThen(shootAlgae())
                .andThen(reset());
            }
            else{
                pivotState = changePivotPos(Constants.AlgaeShooter.PROCESSOR_POS)
                .andThen(Commands.waitSeconds(Constants.AlgaeShooter.REV_UP_TIME))
                .andThen(shootAlgae())
                .andThen(reset());
            }
        }

        //Use a switch-case to determine what the wheels should do at each direction state
        wheelState = switch(state.direction){
            case OFF -> idleWheels();
            case REVERSE -> shootAlgae();
            case FORWARD -> intakeAlgae();
        };
        //Will return the command to change the pivot, which will then follow with the command to change the wheel
        return pivotState.andThen(wheelState);
    }

}