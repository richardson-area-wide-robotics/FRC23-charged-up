// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PIDGains;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kControllerDeadband = 0.1;
  }
  public static final class SwerveDriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    /*
     * Chassis configuration
     */
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // The kinematics for the robot drivetrain
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    /*
     * Spark Max and encoder constents for MAXSwerve modules
     */
    public static final class FrontLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 2;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 1;
      // The Angular offset in radians for the steer encoder in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      // creating the swerve module constants for the front left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kFrontLeftChassisAngularOffset);
    }

    public static final class FrontRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 8;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 7;
      // The Angular offsets in radians for the steer encoder
      public static final double kFrontRightChassisAngularOffset = 0;
      // creating the swerve module constants for the front right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kFrontRightChassisAngularOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 4;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 3;
      // The Angular offsets in radians for the steer encoder
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      // creating the swerve module constants for the back left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kBackLeftChassisAngularOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class BackRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 6;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 5;
      // The Angular offsets in radians for the steer encoder
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;
      // creating the swerve module constants for the back right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kBackRightChassisAngularOffset);
    }

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kFreeSpeedRpm = 5676.0;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0.0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final PIDGains kDrivingPIDGains =
        new PIDGains(0.04, 0, 0); // TODO: tune values for the driving motor
    public static final double kDrivingFF = 1.0 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1.0;
    public static final double kDrivingMaxOutput = 1.0;

    public static final PIDGains kTurningPIDGains =
        new PIDGains(4, 0, 0.075); // TODO: tune values for the turning motor
    public static final double kTurningFF = 0.0; // TODO: tune values for Feed Forward
    public static final double kTurningMinOutput = -1.0;
    public static final double kTurningMaxOutput = 1.0;

    
    public static final PIDGains kVisionTurningPIDGains =
        new PIDGains(1.0, 0, 0.01); // TODO: tune values for Vision auto-turning

    public static final double MAX_LOCKED_ON_SPEED = 0.33;
    
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 10; // amps
  }
  public static final class AutoConstants {
     public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  
  public static final class Intake {
    public static final float kRLimit  = -4.0f;
    public static final float kFLimit = 0.0f;
    public static final double kForwardSpeed = 0.7;
    public static final double kReverseSpeed = -1.0;
    public static final boolean kIntakeInverted = true;
    public static final int kIntakeID = 11;
    public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
    public static final int kIntakeCurrentLimit = 40; // amps
    public static final double kIntakeOffset = 0.0; // TODO: set offset
    public static final Double kIntakeOpen = -3.5;
    public static final Double kIntakeClosed = -1.4;
    public static final PIDGains kIntakePIDGains = new PIDGains(0.1, 0.0, 0.0);
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }
   public static final class ArmConstants {
    // Arm limits
    public final static float REVERSE_LIMIT = 0.0f;
    public final static float FORWARD_LIMIT = 43.5f;
    public final static double REVERSE_SPEED = 0.5;
    public final static double FORWARD_SPEED = 0.5;

    // doubles for arm positions
    public final static double INTAKE_ARM_GROUND = 0.0;
    public final static double INTAKE_ARM_SHELF = 0.0;
    public final static double SCORING_ARM_LOW = 0.0;
    public final static double SCORING_ARM_MID = 0.0;
    public static final Double INTAKE_ARM_STOWED = 0.0;

    /* Spark max constants */ 
    // CAN ID
    public final static int LEFT_MOTOR_CAN_ID = 9;
    public final static int RIGHT_MOTOR_CAN_ID = 10;
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = true; 
    public static final double kArmEncoderPositionFactor = 0.0;
    public static final double kArmEncoderVelocityFactor = 0.0;
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final int kArmMotorCurrentLimit = 40;
    public static final int kMovingArmMotorCurrentLimit = 60;  

    // PID constants using custom PID gains class //TODO: tune PID values
    public static final PIDGains ARM_PID_GAINS = new PIDGains(1.0, 0.0, 0.0);
    public static final double ARM_FF = 0.0;
    public static final double ARM_MIN_OUTPUT = -1.0;
    public static final double ARM_MAX_OUTPUT = 1.0;
    
    public static final double ARM_ENCODER_OFFSET = 0;
  }

}
