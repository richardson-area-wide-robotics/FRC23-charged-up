package frc.robot.subsystems.arm;

//importing libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// extend SubsystemBase class into the Arm class?
public class Arm {

    // making variable under the CANSparkMax class
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    //variables to set limits, speeds, and ramp-rate
    private final float REVERSE_LIMIT = (float) 0;
    private final float FORWARD_LIMIT = (float) 0;
    private final double REVERSE_SPEED = 0;
    private final double FORWARD_SPEED = 0;
    private final double RAMPRATE = 0;

    //variables to set the arm position when scoring and intaking
    private armPosition lowScore = armPosition.SCORING_ARM_POSITION_LOW;
    private armPosition midScore = armPosition.SCORING_ARM_POSITION_MID;
    private armPosition groundIn = armPosition.INTAKE_ARM_POSITION_GROUND;
    private armPosition shelfIn = armPosition.INTAKE_ARM_POSITION_SHELF;

    // enum sets unchangeable variables, here it sets the arm height for intaking and scoring game objects
    enum armPosition {
    INTAKE_ARM_POSITION_GROUND,
    INTAKE_ARM_POSITION_SHELF,
    SCORING_ARM_POSITION_MID,
    SCORING_ARM_POSITION_LOW
    }

    //setting up CAN IDs for the motors
    public Arm(int rightMotorCANID, int leftMotorCANID) {

    //motor type for right motor
    this.rightMotor = new CANSparkMax(rightMotorCANID, MotorType.kBrushless);

    // comment below is for incase the motors need to be reversed
    // this.rightMotor.setInverted(invertRightMotor);

    //setting soft limits (soft limits keep the motor running when it hits the limit instead of braking)
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, FORWARD_LIMIT);
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, REVERSE_LIMIT);

    //setting the ramp-rate for open loop modules (the max amount the motor controller's output is allowed to change)
    this.rightMotor.setOpenLoopRampRate(RAMPRATE);

    //setting the idle mode setting for the SparkMax, here it is set to brake when idle
    this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);


    //motor type for left motor
    this.leftMotor = new CANSparkMax(leftMotorCANID, MotorType.kBrushless);

    // comment below is for incase the motors need to be reversed
    // this.leftMotor.setInverted(invertLeftMotor);

    this.leftMotor.follow(this.rightMotor, true);
  }

  //setting functions of runToScore and runToIntake
  public void runToScoreLow() {
    this.rightMotor.set(FORWARD_SPEED);
    this.lowScore = armPosition.SCORING_ARM_POSITION_LOW;
  }

  public void runToScoreMid() {
    this.rightMotor.set(FORWARD_SPEED);
    this.midScore = armPosition.SCORING_ARM_POSITION_MID;
  }

  public void runToIntakeGround() {
    this.rightMotor.set(REVERSE_SPEED);
    this.groundIn = armPosition.INTAKE_ARM_POSITION_GROUND;
  }

  public void runToIntakeShelf() {
    this.rightMotor.set(REVERSE_SPEED);
    this.shelfIn = armPosition.INTAKE_ARM_POSITION_SHELF;
  }

  //getting and positions, speed, and limits and returning them to us
  public double getPosition() {
    return this.rightMotor.getEncoder().getPosition();
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


  //confusion, code works for now, but need to find out how to make runToScore and runToIntake choose between--
  //--low and mid, and ground and shelf based on player input
  public void moveArmToPosition(armPosition position) {
    if (position == armPosition.INTAKE_ARM_POSITION_GROUND) {
      this.runToScoreMid();
      this.runToScoreLow();
      }
      else if (position == armPosition.INTAKE_ARM_POSITION_SHELF) {
      this.runToScoreMid();
      this.runToScoreLow();
      }
      else if (position == armPosition.SCORING_ARM_POSITION_LOW) {
      this.runToIntakeGround();
      this.runToIntakeShelf();
      }
      else if (position == armPosition.SCORING_ARM_POSITION_MID) {
      this.runToIntakeShelf();
      this.runToIntakeGround();
      }
    } 
}
