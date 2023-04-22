package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

private CANSparkMax intakeMotor;
public boolean mode;
private DigitalInput sensor;
public boolean desiredM;

public Intake() {

  intakeMotor = new CANSparkMax(Constants.Intake.kIntakeID, MotorType.kBrushless);
  sensor = new DigitalInput(0);

  // set intake motor to factory defaults for if we ever want to switch them out 
  intakeMotor.restoreFactoryDefaults();

  
  // set intake basic values 
  intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeCurrentLimit);
  intakeMotor.setInverted(Constants.Intake.kIntakeInverted);
  intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);

  intakeMotor.burnFlash();

  this.mode = false;
}


/**
 * Returns the speed of the intake motor
 * @return speed
 */
public double getSpeed() {
  return this.intakeMotor.get();
}

/**
 * Stops the intake
 */
public Command stop(){
  return new RunCommand(()-> this.setIntakeSpeed(0), this);
}

/**
 * Intakes based on the speed given
 * @param speed the speed of the intake
 */
public void setIntakeSpeed(double speed){
  intakeMotor.set(speed);
}

/**
 * Toggles Mode of Intake
 */
public void toggleMode(){
  mode = !mode;
}

/**
 * sets the Mode of the Intake
 */
public void setMode(boolean desiredMode){
  mode = desiredMode;
}

public boolean setXMode(boolean desiredMode){
  return desiredM = desiredMode;
}

/** 
 * Returns the mode of the intake; true for Cube mode, false for Cone mode; default is false
 * @return mode
 */
public boolean getMode(){
  return mode;
}

/**
 * Intaking command for the intake
 * @param speed the speed of the intake
 * @param intakingMode As cube intaking is positive and cone intaking is negative this command will invert based on the mode
 */
public Command manipulatorCommand(double speed, boolean intakingMode){
  if(!getMode()){
    return new RunCommand(()->  this.setIntakeSpeed(speed), this);
  } else {
    return new RunCommand(()-> this.setIntakeSpeed(-speed), this);
  }
}

/**
 * Intaking/outtaking Command for cones and cubes
 * Will take in a speed, if mode is true then it will invert the speed to intake cube, if mode is false then it will use the normal speed to intake cone
 * Also if the mode is set to false and original speed is negative then it will set the speed to -0.25 for cone outtaking
 * @param speed the speed of the intake
 */
public Command manipulatorCommand(double speed){
  if(!getMode()){
    return new RunCommand(()-> this.setIntakeSpeed(speed), this);
  } else {
    if(speed < 0){
      return new RunCommand(()-> this.setIntakeSpeed(-0.25), this);
    } else {
      return new RunCommand(()-> this.setIntakeSpeed(-speed), this);
    }
  }
}

/**
 * Intaking Idle command for the intake 
 * intakes based of the mode given from this class, if mode is true then it will idle as a cube, if mode is false then it will idle as a cone
 */
public Command idle(){
  if(!mode){
    return new RunCommand(()-> this.setIntakeSpeed(0.1), this);
  } else  {
    return new RunCommand(()-> this.setIntakeSpeed(-0.1), this);
  }
}


/**
 * Returns the output current of the intake motor
 * @return outputCurrent
 */
public double outputCurrent(){
  return intakeMotor.getOutputCurrent();
}

/**
 * Returns the sensor data from the intake
 * @return sensor
 */
public boolean getSensorData(){
  return sensor.get();
}

@Override 
public void periodic(){
  if (mode != desiredM){
    mode = desiredM;
  }
  SmartDashboard.putNumber("Output current", outputCurrent());
}

}