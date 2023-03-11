package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.EnumMap;

public class Arm extends SubsystemBase {

  // Shoulder Motors
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  // Elbow Motors
  private CANSparkMax elbowMotor;

  // PID Controllers
  private PIDController armPID;
  private PIDController elbowPID;  
  private SparkMaxPIDController armPIDController;
  private SparkMaxPIDController elbowPIDController;  

  // Feedforward
  private ArmFeedforward elbowFF;
  private ArmFeedforward armFF;

  // Encoders
  private AbsoluteEncoder armEncoder;
  private AbsoluteEncoder elbowEncoder;

  // Arm Positions
  private double currentArmPosition;
  private double currentElbowPosition;

  // last arm positions
  private double lastArmPosition;
  private double lastElbowPosition;


  // set up the arm congfiguration
  public void armConfig(CANSparkMax motor, AbsoluteEncoder enc){
    // restore factory defaults
    motor.restoreFactoryDefaults();
    // set motor basic values
    motor.setIdleMode(Constants.ArmConstants.kMotorIdleMode);
    // motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    // motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    // set the arm PID controllers
    motor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
    armPIDController = motor.getPIDController();
    armPIDController.setFeedbackDevice(enc);
    armPIDController.setPositionPIDWrappingEnabled(false);
    armPID = new PIDController(Constants.ArmConstants.ARM_PID_GAINS.P, Constants.ArmConstants.ARM_PID_GAINS.I, Constants.ArmConstants.ARM_PID_GAINS.D);
    armPIDController.setP(armPID.getP());
    armPIDController.setI(armPID.getI());
    armPIDController.setD(armPID.getD());
    armPIDController.setOutputRange(
        Constants.ArmConstants.MIN_OUTPUT, Constants.ArmConstants.MAX_OUTPUT);
  //       motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
  //       motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);
// motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ArmConstants.ARM_FORWARD_LIMIT);
//     motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.ArmConstants.ARM_REVERSE_LIMIT);
  
  }

  // set up the elbow congfiguration
  public void elbowConfig(CANSparkMax motor, AbsoluteEncoder enc){
 // restore factory defaults
 motor.restoreFactoryDefaults();
 // set motor basic values
 motor.setIdleMode(Constants.ArmConstants.kMotorIdleMode);
 // motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
 // motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
  motor.setSmartCurrentLimit(Constants.ArmConstants.kElbowMotorCurrentLimit);
  elbowPIDController = motor.getPIDController();
  elbowPIDController.setFeedbackDevice(enc);
  elbowPIDController.setPositionPIDWrappingEnabled(false);
  elbowPID = new PIDController(Constants.ArmConstants.ELBOW_PID_GAINS.P, Constants.ArmConstants.ELBOW_PID_GAINS.I, Constants.ArmConstants.ELBOW_PID_GAINS.D);
  elbowPIDController.setP(elbowPID.getP());
  elbowPIDController.setI(elbowPID.getI());
  elbowPIDController.setD(elbowPID.getD());
  elbowPIDController.setOutputRange(
      Constants.ArmConstants.MIN_OUTPUT, Constants.ArmConstants.MAX_OUTPUT);
  //     motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
  // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);
      // motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ArmConstants.ELBOW_FORWARD_LIMIT);
      // motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.ArmConstants.ELBOW_REVERSE_LIMIT);
  }

  public Arm() {
    // motor type for right motor
    leftMotor = new CANSparkMax(Constants.ArmConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.ArmConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    elbowMotor = new CANSparkMax(Constants.ArmConstants.ELBOW_MOTOR_CAN_ID, MotorType.kBrushless);

    // setting the Absolute Encoder for the SparkMax
    armEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
    elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second
    armEncoder.setPositionConversionFactor(Constants.ArmConstants.kArmEncoderPositionFactor);
    armEncoder.setVelocityConversionFactor(Constants.ArmConstants.kArmEncoderVelocityFactor);

    elbowEncoder.setPositionConversionFactor(Constants.ArmConstants.kElbowEncoderPositionFactor);
    elbowEncoder.setVelocityConversionFactor(Constants.ArmConstants.kElbowEncoderVelocityFactor);

    // this.resetArmPosition();

    // setting the motor configuration
    // armConfig(leftMotor, armEncoder, false);
    armConfig(rightMotor, armEncoder);
    elbowConfig(elbowMotor, elbowEncoder);

    rightMotor.setInverted(Constants.ArmConstants.RIGHT_ARMMOTOR_INVERTED);
    elbowMotor.setInverted(true);

    // setting the idle mode setting for the SparkMax, here it is set to brake when idle
    leftMotor.follow(this.rightMotor, true);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
    elbowMotor.burnFlash();

    // armPositions.put(armPosition.INTAKE_ARM_POSITION_GROUND,  Constants.ArmConstants.INTAKE_ARM_GROUND);
    // armPositions.put(armPosition.INTAKE_ARM_POSITION_SHELF,  Constants.ArmConstants.INTAKE_ARM_SHELF);
    // armPositions.put(armPosition.SCORING_ARM_POSITION_LOW,  Constants.ArmConstants.SCORING_ARM_LOW);
    // armPositions.put(armPosition.SCORING_ARM_POSITION_MID, Constants.ArmConstants. SCORING_ARM_MID);
    // armPositions.put(armPosition.INTAKE_ARM_POSITION_STOWED,  Constants.ArmConstants.INTAKE_ARM_STOWED);

    this.currentArmPosition = Constants.ArmConstants.ARM_STOWED;
    this.currentElbowPosition = Constants.ArmConstants.ELBOW_STOWED;
    this.lastArmPosition = currentArmPosition;
    this.lastElbowPosition = currentElbowPosition;

    elbowFF = Constants.ArmConstants.ELBOW_MOTOR_FEEDFORWARD;
    armFF = Constants.ArmConstants.ARM_MOTOR_FEEDFORWARD;
  }

  // getting relative encoder position of the arm
  public double getPosition() {
    return this.leftMotor.getEncoder().getPosition();
  }

  public double getLastArmPosition(){
    return lastArmPosition;
  }

  public double getArmPosition(){
    return currentArmPosition;
  }

  public double getArmAbsoluteEncoder(){
    return armEncoder.getPosition(); //- Units.degreesToRadians(50);
  }

  public double getElbowAbsoluteEncoder(){
    return elbowEncoder.getPosition();
  }

  public double outputcurrent(){
    return elbowMotor.getOutputCurrent();
  }

  public double outputleftcurrent(){
    return leftMotor.getOutputCurrent();
  }

  public double outputrightcurrent(){
    return rightMotor.getOutputCurrent();
  }

  public double speed(){
    return elbowEncoder.getVelocity();
  }

  // set the arm speed
  public void setSpeed(double speed) {
    this.leftMotor.set(speed);
  }

  // get the arm speed - returns in RPM (revolutions per minute)
  public double getSpeed() {
    return this.armEncoder.getVelocity();
  }

  public void moveElbowPosition(int armPosition){
    if(armPosition == 0){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_PICK_UP_SHELF;
    }
    if(armPosition == 1){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_PICK_UP_TCONE;
    }
    else if(armPosition == 2){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_PICK_UP_CONE;
    }
    else if(armPosition == 3){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_PICK_UP_CUBE;
    }
    else if(armPosition == 4){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_SCORE_CUBE_LOW;
    }
    else if(armPosition == 5){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_SCORE_CONE_LOW;
    }
    else if(armPosition == 6){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_SCORE_CONE_MID;
    }
    else if(armPosition == 7){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_SCORE_CUBE_MID;
    }
    else if(armPosition == 8){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_SCORE_CONE_HIGH;
    }
    else if(armPosition == 9){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_SCORE_CUBE_HIGH;
    }
    else if(armPosition == 10){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_STOWED;
    }
    else if(armPosition == 11){
      this.currentElbowPosition = Constants.ArmConstants.ELBOW_IDLE;
    }
  }

  // arm movement controls using PID loop
  public void moveArmToPosition(int armPosition) {

    if(armPosition == 0){
      this.currentArmPosition = Constants.ArmConstants.ARM_PICK_UP_SHELF;
    }
    if(armPosition == 1){
      this.currentArmPosition = Constants.ArmConstants.ARM_PICK_UP_TCONE;
    }
    else if(armPosition == 2){
      this.currentArmPosition = Constants.ArmConstants.ARM_PICK_UP_CONE;
    }
    else if(armPosition == 3){
      this.currentArmPosition = Constants.ArmConstants.ARM_PICK_UP_CUBE;
    }
    else if(armPosition == 4){
      this.currentArmPosition = Constants.ArmConstants.ARM_SCORE_CUBE_LOW;
    }
    else if(armPosition == 5){
      this.currentArmPosition = Constants.ArmConstants.ARM_SCORE_CONE_LOW;
    }
    else if(armPosition == 6){
      this.currentArmPosition = Constants.ArmConstants.ARM_SCORE_CONE_MID;
    }
    else if(armPosition == 7){
      this.currentArmPosition = Constants.ArmConstants.ARM_SCORE_CUBE_MID;
    }
    else if(armPosition == 8){
      this.currentArmPosition = Constants.ArmConstants.ARM_SCORE_CONE_HIGH;
    }
    else if(armPosition == 9){
      this.currentArmPosition = Constants.ArmConstants.ARM_SCORE_CUBE_HIGH;
    }
    else if(armPosition == 10){
      this.currentArmPosition = Constants.ArmConstants.ARM_STOWED;
    }
  }

  public void setArmPosition(double position) {
    currentArmPosition = position;
  }

  public void setElbowPosition(double position) {
    currentElbowPosition = position;
  }

  public void getSparkStatus(IdleMode mode){
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode);
    elbowMotor.setIdleMode(mode);
  }

  public void setShoulderPower(double power){
    rightMotor.set(power);
  }

  public void setElbowPower(double power){
    elbowMotor.set(power);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    armPIDController.setReference(currentArmPosition, ControlType.kPosition);
    /* , 1, armFF.calculate(currentElbowPosition, armPID.getSetpoint().velocity)*/
    SmartDashboard.putNumber("outputcurrent for elbow", outputcurrent());
    SmartDashboard.putNumber("outputcurrent for left", outputleftcurrent());
    SmartDashboard.putNumber("outputcurrent for right", outputrightcurrent());

    elbowPIDController.setReference(currentElbowPosition, ControlType.kPosition/* , 1, elbowFF.calculate(elbowPID.getSetpoint().position, elbowPID.getSetpoint().velocity)*/);

    lastArmPosition = currentArmPosition;
    lastElbowPosition = currentElbowPosition;
  }
}
