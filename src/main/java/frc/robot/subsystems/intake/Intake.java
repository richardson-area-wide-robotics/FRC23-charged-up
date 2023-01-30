package frc.robot.subsystems.intake;

import java.util.EnumMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

private CANSparkMax intakeMotor;
private AbsoluteEncoder intakeEncoder;
private RelativeEncoder intakeRelativeEncoder;
private SparkMaxPIDController intakePIDController;

 // enum sets unchangeable variables, here it sets the arm height for intaking and scoring game
  // objects
  public enum intakePosition {
    INTAKE_CLOSED, 
    INTAKE_OPEN
  }

// Map of intake positions
EnumMap<intakePosition, Double> intakePositions = new EnumMap<>(intakePosition.class);

// variable to set the arm position when scoring and intaking
private intakePosition currentIntakePosition = intakePosition.INTAKE_CLOSED;

public Intake() {

  intakeMotor = new CANSparkMax(Constants.Intake.kIntakeID, MotorType.kBrushless);

  // set intake motor to factory defaults for if we ever want to switch them out 
  intakeMotor.restoreFactoryDefaults();

  //intakeEncoder = intakeMotor.getAbsoluteEncoder(Type.kDutyCycle);
  intakeRelativeEncoder = intakeMotor.getEncoder();

  // set intake basic values 
  intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeCurrentLimit);
  intakeMotor.setInverted(Constants.Intake.kIntakeInverted);
  intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);

  // set the intake PID controllers
  intakePIDController = intakeMotor.getPIDController();
  // intakePIDController.setFeedbackDevice(intakeEncoder);
  intakePIDController.setFeedbackDevice(intakeRelativeEncoder);
  intakePIDController.setP(Constants.Intake.kIntakePIDGains.P);
  intakePIDController.setOutputRange(
      Constants.Intake.kMinOutput, Constants.Intake.kMaxOutput);

  // enable and set intake motor forward and reverse limit
  intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
  intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
  intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.Intake.kFLimit);
  intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.Intake.kRLimit);

  intakeMotor.burnFlash();

  intakePositions.put(intakePosition.INTAKE_OPEN,  Constants.Intake.kIntakeOpen);
  intakePositions.put(intakePosition.INTAKE_CLOSED,  Constants.Intake.kIntakeClosed);

  setDefaultCommand(new RunCommand(this::idle, this));
}

public double getSpeed() {
  return intakeMotor.getAppliedOutput();
}

// returns the raw position of the intake encoder in rotations
public double getPosition() {
  return intakeEncoder.getPosition();
}

// // returns the absolute rotation of the intake Absolute Encoder without the offset
// public Rotation2d getAbsoluteRotation() {
//   return new Rotation2d(getPosition() - Constants.Intake.kIntakeOffset);
// }

// sets the intake motor to 0 to stop all movement 
public void idle() {
  intakeMotor.set(0.0);
}

public void MunaulPickup(){
  intakeMotor.set(Constants.Intake.kForwardSpeed);
}

public void ManualDrop() {
  intakeMotor.set(Constants.Intake.kReverseSpeed);
}

// returns the setpoint of the intake PID controller; can be used to check intake pid contoller
public double getSetPoint(){
  return intakePositions.get(currentIntakePosition);
}

/* Toggles the state of the intake between clamped and open, using enum positions - defualt will be closed, with set reference method running the robot in closed loop  */
public void toggleIntake() {
  if (currentIntakePosition == intakePosition.INTAKE_CLOSED) {
    currentIntakePosition = intakePosition.INTAKE_OPEN;
  } else {
    currentIntakePosition = intakePosition.INTAKE_CLOSED;
  }
  intakePIDController.setReference(intakePositions.get(currentIntakePosition), CANSparkMax.ControlType.kPosition);
}

}
