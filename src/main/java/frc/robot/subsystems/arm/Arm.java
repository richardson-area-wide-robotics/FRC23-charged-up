package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

import java.util.EnumMap;

// extend SubsystemBase class into the Arm class?
public class Arm {

  // making variable under the CANSparkMax class
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private Encoder armEncoder;

  // enum sets unchangeable variables, here it sets the arm height for intaking and scoring game
  // objects
  public enum armPosition {
    SCORING_ARM_POSITION_LOW,
    SCORING_ARM_POSITION_MID,
    INTAKE_ARM_POSITION_GROUND,
    INTAKE_ARM_POSITION_SHELF
  }



  // Map of arm positions named armPositions
  EnumMap<armPosition, Double> armPositions = new EnumMap<>(armPosition.class);

  // variable to set the arm position when scoring and intaking
  private armPosition currentArmPosition = armPosition.INTAKE_ARM_POSITION_GROUND;

  // setting up CAN IDs for the motors
  public Arm(int rightMotorCANID, int leftMotorCANID, int EncoderPort1, int EncoderPort2) {

    // motor type for right motor
    this.rightMotor = new CANSparkMax(rightMotorCANID, MotorType.kBrushless);
    this.leftMotor = new CANSparkMax(leftMotorCANID, MotorType.kBrushless);

    this.armEncoder = new Encoder(EncoderPort1, EncoderPort2);
    
    // restoring defaults for spark max for if we need to switch them out
    this.rightMotor.restoreFactoryDefaults();
    this.leftMotor.restoreFactoryDefaults();

    // setting soft limits (soft limits keep the motor running when it hits the limit instead of
    // braking)
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ArmConstants.FORWARD_LIMIT);
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,  Constants.ArmConstants.REVERSE_LIMIT);

    // setting the idle mode setting for the SparkMax, here it is set to brake when idle
    this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    this.rightMotor.setInverted(Constants.ArmConstants.RIGHT_MOTOR_INVERTED);

    this.leftMotor.follow(this.rightMotor, Constants.ArmConstants.LEFT_MOTOR_INVERTED);

    // setting the PID loop for the arm
    this.rightMotor.getPIDController().setP( Constants.ArmConstants.PID_GAINS.P);
    this.rightMotor.getPIDController().setI( Constants.ArmConstants.PID_GAINS.I);
    this.rightMotor.getPIDController().setD( Constants.ArmConstants.PID_GAINS.D);

    this.rightMotor.burnFlash();
    this.leftMotor.burnFlash();

    armPositions.put(armPosition.INTAKE_ARM_POSITION_GROUND,  Constants.ArmConstants.INTAKE_ARM_GROUND);
    armPositions.put(armPosition.INTAKE_ARM_POSITION_SHELF,  Constants.ArmConstants.INTAKE_ARM_SHELF);
    armPositions.put(armPosition.SCORING_ARM_POSITION_LOW,  Constants.ArmConstants.SCORING_ARM_LOW);
    armPositions.put(armPosition.SCORING_ARM_POSITION_MID, Constants.ArmConstants. SCORING_ARM_MID);
  }

  // setting functions of runToScore and runToIntake
  public void runToScoreLow() {
    this.rightMotor.set( Constants.ArmConstants.FORWARD_SPEED);
    this.currentArmPosition = armPosition.SCORING_ARM_POSITION_LOW;
  }

  public void runToScoreMid() {
    this.rightMotor.set( Constants.ArmConstants.FORWARD_SPEED);
    this.currentArmPosition = armPosition.SCORING_ARM_POSITION_MID;
  }

  public void runToIntakeGround() {
    this.rightMotor.set( Constants.ArmConstants.REVERSE_SPEED);
    this.currentArmPosition = armPosition.INTAKE_ARM_POSITION_GROUND;
  }

  public void runToIntakeShelf() {
    this.rightMotor.set( Constants.ArmConstants.REVERSE_SPEED);
    this.currentArmPosition = armPosition.INTAKE_ARM_POSITION_SHELF;
  }

  // getting and positions, speed, and limits and returning them to us
  public double getPosition() {
    return this.rightMotor.getEncoder().getPosition(); // this needs to change as we will be using external encoder for arm
  }

  public double getMeasurement() {
    return this.getPosition();
  }

  public Boolean atForwardLimit() {
    return this.rightMotor.getFault(CANSparkMax.FaultID.kSoftLimitFwd);
  }

  public double getSpeed() {
    return this.rightMotor.get();
  }

  public Boolean atReverseLimit() {
    return this.rightMotor.getFault(CANSparkMax.FaultID.kSoftLimitRev);
  }

  // arm movement controls using PID loop
  public void moveArmToPosition() {
    rightMotor
        .getPIDController()
        .setReference(armPositions.get(currentArmPosition), CANSparkMax.ControlType.kPosition);
  }
}
