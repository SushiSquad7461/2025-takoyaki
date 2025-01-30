
package frc.robot.subsystems;

//Imports
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkMax;

//Class for the Algae Intake
abstract public class AlgaeIntake extends SubsystemBase {
    //Variables to hold the moter and encoder objects
    protected final SparkMax intakeMotor;
    protected final SparkMax wheelMotor;
    private final AbsoluteEncoder intakeEncoder;

    //Variable holding the Algae Intake object
    private static AlgaeIntake instance;

    //Variable to check the arm feed forward of intake
    private final ArmFeedforward intakeFeedforward;

;
    
    //Constructor
    public AlgaeIntake(){

        //Initializing all of the objects for the motors and encoders 
        intakeEncoder = new AbsoluteEncoder(Constants.AlgaeIntake.ENCODER_ID, Constants.AlgaeIntake.ENCODER_OFFSET, false);
        intakeMotor = Constants.AlgaeIntake.INTAKE_CONFIG.createSparkMax();
        wheelMotor = Constants.AlgaeIntake.INTAKE_CONFIG.createSparkMax();
        MotorHelper.setDegreeConversionFactor(intakeMotor, Constants.AlgaeIntake.INTAKE_GEAR_RATIO);
        


    }
    //Ressetting the motors 
    public void reset() {
        intakeMotor.getEncoder().setPosition(intakeEncoder.getNormalizedPosition());
        wheelMotor.getEncoder().setPosition(intakeEncoder.getNormalizedPosition());

    }

    //Command to raise the intake up to resting 
    public Command raiseIntake(){
        //return adjustArmAngle(Constants.AlgaeIntake.RAISED);
        return null;
    }

    //Command to lower intake 
    public Command lowerIntake(){
       // return adjustArmAngle(Constants.AlgaeIntake.LOWER);
       return null;
    }

    //Command to spin the intake wheels
    public Command spinIntake(){
        //return spinWheels(Constants.AlgaeIntake.WHEEL_SPEED);
        return null;
    }

     //Command to stop spinning of intake wheels
    public Command stopIntakeSpin(){
        //return spinWheels(0);
        return null;
    }

    //Command to change the angle of the arm
    public Command adjustArmAngle(double pos){
        return null;
    }

    public Command spinWheels(double speed){
        return null;
    }

    public static AlgaeIntake getInstance() {
        if (instance == null) {
          //  return new AlgaeIntake();
        }
        return instance;
    }
}
