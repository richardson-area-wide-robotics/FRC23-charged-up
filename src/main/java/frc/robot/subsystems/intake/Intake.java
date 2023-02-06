package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

private CANSparkMax intakeMotor;

public Intake() {

  intakeMotor = new CANSparkMax(Constants.Intake.kIntakeID, MotorType.kBrushless);

  // set intake motor to factory defaults for if we ever want to switch them out 
  intakeMotor.restoreFactoryDefaults();

  // set intake basic values 
  intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeCurrentLimit);
  intakeMotor.setInverted(Constants.Intake.kIntakeInverted);
  intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);

  intakeMotor.burnFlash();

  setDefaultCommand(new RunCommand(this::idle, this));
}


// Returns the speed of the NEO550 motor
public double getSpeed() {
  return this.intakeMotor.get();
}

// sets the intake motor to 0 to stop all movement 
public void idle() {
  intakeMotor.set(0.0);
}

// sets the motor to intake
public void intake(){
  intakeMotor.setVoltage(Constants.Intake.kIntakeSpeed);
}

// sets the motor to outaking - also used for tiped over cone intaking 
public void outake(){
  intakeMotor.setVoltage(Constants.Intake.kOutakeSpeed);
}

}
