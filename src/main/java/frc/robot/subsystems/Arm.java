package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// extending SubsystemBase class into the Arm class
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

        //variables to set the arm position when scoring 
        private armPosition rosemi = armPosition.SCORING_ARM_POSITION_MID;
        private armPosition fortnite = armPosition.SCORING_ARM_POSITION_LOW;

        // enum sets unchangeable variables, here it sets the arm height for intaking and scoring game objects
        enum armPosition {
        INTAKE_ARM_POSITION,
        SCORING_ARM_POSITION_MID,
        SCORING_ARM_POSITION_LOW
      }

      //setting up CAN IDs for the motors
      public Arm(int rightMotorCANID, int leftMotorCANID) {
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

        public void runToScore() {
            this.rightMotor.set(FORWARD_SPEED);
            this.fortnite = armPosition.SCORING_ARM_POSITION_LOW;
          }

          public void runToScore() {
            this.rightMotor.set(FORWARD_SPEED);
            this.rosemi = armPosition.SCORING_ARM_POSITION_MID;
          }
        
          public void runToIntake() {
            this.rightMotor.set(REVERSE_SPEED);
            this. = armPosition.INTAKE_ARM_POSITION;
          }
        
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
        
          public void moveArmToPosition(armPosition position) {
            if (position == armPosition.INTAKE_ARM_POSITION) {
              this.runToScore();
            } else if (position == armPosition.SCORING_ARM_POSITION) {
              this.runToIntake();
            }
          }
        
          public void toggleArmPosition() {
            if (this.m_Position == armPosition.INTAKE_ARM_POSITION) {
              this.runToScore();
            } else {
              this.runToIntake();
            }
          }
  }
}
