package frc.robot.subsystems.intake;

//fortnite balls go brrrrrrrrr you just got coconut malled!

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Intake {

intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeCurrentLimit);
intakeMotor.setInverted(false);
intakeMotor.setIdleMode(Constants.Intake.kBrake);

intakeMotor.restoreFactoryDefaults();

CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);
//do we use a NEO550? i dunno but sure

  static final int INTAKE_CURRENT_LIMIT_A = 25;
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
  static final double INTAKE_OUTPUT_POWER = 1.0;
  static final double INTAKE_HOLD_POWER = 0.07;


public void setIntakeMotor(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
// because I like seeing numbers
}

intake.setSmartCurrentLimit(Constants.Intake.kIntakeCurrentLimit);
  intakeMotor.setInverted(Constants.Intake.kIntakeInverted);
  intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);

  intakePIDController = intakeMotor.getPIDController();
  intakePIDController.setFeedbackDevice(intakeRelativeEncoder);
  intakePIDController.setP(Constants.Intake.kIntakePIDGains.P);
  intakePIDController.setOutputRange(
  Constants.Intake.kMinOutput, Constants.Intake.kMaxOutput);

intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.Intake.kFLimit,2);
intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.Intake.kRLimit,1);

intake.burnFlash();

//I believe I do not need to set positions because this is a roller, not a claw

public double getSpeed() {
    return intakeMotor.getAppliedOutput();
}

public void Idle() { 
    intakeMotor.set(0.0);
}

public void In(){
    intakeMotor.set(Constants.Intake.kForwardSpeed);
}
  
public void Out() {
    intakeMotor.set(Constants.Intake.kReverseSpeed);
}

} //ooga booga
