package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

private CANSparkMax intakeMotor;
private boolean mode;
private DigitalInput sensor;

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

  setDefaultCommand(new RunCommand(this::stop, this));

  // setDefaultCommand(idle());

  this.mode = false;
}


// Returns the speed of the NEO550 motor
public double getSpeed() {
  return this.intakeMotor.get();
}

// sets the intake motor to 0 to stop all movement 
public void idle(double speed) {
  intakeMotor.set(speed);
}

/**
 * Stops the intake
 */
public void stop(){
  intakeMotor.set(0.0);
}

// sets the motor to intake
public void manipulates(double speed){
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
 * @param mode As cube intaking is positive and cone intaking is negative this command will invert based on the mode
 */
public Command manipulatorCommand(double speed, boolean mode){
  if(mode){
    return new RunCommand(()-> this.manipulates(speed), this);
  } else {
    return new RunCommand(()-> this.manipulates(-speed), this);
  }
}

/**
 * Intaking Idle command for the intake 
 * intakes based of the mode given from this class, if mode is true then it will idle as a cube, if mode is false then it will idle as a cone
 */
public Command idle(){
  if(!mode){
    return new RunCommand(()-> this.manipulates(0.1), this);
  } else {
    return new RunCommand(()-> this.manipulates(-0.1), this);
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

/* Manipulates the smart current limit to act as a "hard" stop for intaking */
public void setSmartCurrentLimit(){}

@Override 
public void periodic(){
  SmartDashboard.putNumber("Output current", outputCurrent());
}

}