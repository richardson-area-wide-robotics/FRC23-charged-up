package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

private CANSparkMax intakeMotor;
private double intaking;
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
}


// Returns the speed of the NEO550 motor
public double getSpeed() {
  return this.intakeMotor.get();
}

// sets the intake motor to 0 to stop all movement 
public void idle(double speed) {
  intakeMotor.set(speed);
}

public void stop(){
  intakeMotor.set(0.0);
}

// sets the motor to intake
public void manipulates(double speed){
  intakeMotor.set(speed);
}

public Command manipulator(double supplier, boolean mode){
  if (!mode){
  intaking = supplier;
} else {
  intaking = supplier * -1;
}

return run(() -> intakeMotor.set(intaking));
}

// returns the current of the motor 
public double outputCurrent(){
  return intakeMotor.getOutputCurrent();
}

public boolean getSensorData(){
  return sensor.get();
}

/* Manipulates the smart current limit to act as a "hard" stop for intaking */
public void setSmartCurrentLimit(){}

@Override 
public void periodic(){
  SmartDashboard.putNumber("Output current for testing", outputCurrent());
  SmartDashboard.putBoolean("sensor", getSensorData());
}

}
