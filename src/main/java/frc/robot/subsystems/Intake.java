package frc.robot.subsystems;

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

  // enable and set intake motor forward and reverse limit
  intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
  intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
  intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.Intake.kFLimit);
  intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.Intake.kRLimit);

  intakeMotor.burnFlash();

  setDefaultCommand(new RunCommand(this::idle, this));
}

public double getSpeed() {
  return intakeMotor.getAppliedOutput();
}

public double getPosition() {
  return intakeMotor.getEncoder().getPosition();
}

public void idle() {
  intakeMotor.set(0);
}

public void pickup(){
  intakeMotor.set(Constants.Intake.kForwardSpeed);
}

public void drop() {
  intakeMotor.set(Constants.Intake.kReverseSpeed);
}

}
