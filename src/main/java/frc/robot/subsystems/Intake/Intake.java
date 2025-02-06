package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Direction;


import frc.robot.Direction;


abstract public class Intake extends SubsystemBase {
    protected final SparkMax intakeMotor;

    public Intake() {
        intakeMotor = Constants.Intake.INTAKE_CONFIG.createSparkMax();
    }

    public Command runIntake() {
        return runOnce(() -> intakeMotor.set(Constants.Intake.INTAKE_SPEED));
    }

    public Command reverseIntake() {
        return runOnce(() -> intakeMotor.set(-1 * Constants.Intake.INTAKE_SPEED));
    }

    public Command stopIntake() {
        return runOnce(() -> intakeMotor.set(0.0));
    }

    public Command lowerIntake() { return Commands.none(); }

    public Command raiseIntake() { return Commands.none(); }


    public Command changeState(Constants.IntakeState newState) {

        //Will check to see if intake is up, if it, lower intake, else, raise intake
        Command pivotCommand = newState.intakeExtended ? lowerIntake() : raiseIntake();

        
        Command intakeCommand = newState.intakeExtended
                //Checks to see if state reverses intake, if it does then reverse intake, if not run intake
                ? (newState.direction == Direction.REVERSED ? reverseIntake() : runIntake())
                //Will stop the intake if not extended
                : stopIntake();
        return pivotCommand.andThen(intakeCommand);
    }
}