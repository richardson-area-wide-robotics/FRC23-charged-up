package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.EnumMap;

// extend SubsystemBase class into the Arm class?
public class Arm {

  // making variable under the CANSparkMax class
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private AbsoluteEncoder armEncoder;
  private SparkMaxPIDController armPIDController;

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
  public Arm() {
    // motor type for right motor
    leftMotor = new CANSparkMax(Constants.ArmConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.ArmConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    
    // restoring defaults for spark max for if we need to switch them out
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    
    // setting the Absolute Encoder for the SparkMax
    armEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second
    armEncoder.setPositionConversionFactor(Constants.ArmConstants.kArmEncoderPositionFactor);
    armEncoder.setVelocityConversionFactor(Constants.ArmConstants.kArmEncoderVelocityFactor);
  
    // Set the PID gains for the driving motor.
    armPIDController = leftMotor.getPIDController();
    armPIDController.setFeedbackDevice(armEncoder);
    armPIDController.setP(Constants.ArmConstants.ARM_PID_GAINS.P);
    armPIDController.setI(Constants.ArmConstants.ARM_PID_GAINS.I);
    armPIDController.setD(Constants.ArmConstants.ARM_PID_GAINS.D);
    armPIDController.setFF(Constants.ArmConstants.ARM_FF);
    armPIDController.setOutputRange(
        Constants.ArmConstants.ARM_MIN_OUTPUT, Constants.ArmConstants.ARM_MAX_OUTPUT);

    leftMotor.setIdleMode(Constants.ArmConstants.kArmMotorIdleMode);
    leftMotor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
    leftMotor.setInverted(Constants.ArmConstants.RIGHT_MOTOR_INVERTED);

    // setting soft limits (soft limits keep the motor running when it hits the limit instead of
    // braking)
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ArmConstants.FORWARD_LIMIT);
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,  Constants.ArmConstants.REVERSE_LIMIT);

    // setting the idle mode setting for the SparkMax, here it is set to brake when idle
    rightMotor.follow(this.leftMotor, Constants.ArmConstants.LEFT_MOTOR_INVERTED);

    leftMotor.burnFlash();
    rightMotor.burnFlash();

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
