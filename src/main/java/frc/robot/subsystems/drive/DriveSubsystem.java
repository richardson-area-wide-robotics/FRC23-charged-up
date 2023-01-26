// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.lib.swerve.MAXSwerve;
import frc.robot.Constants;

public class DriveSubsystem extends MAXSwerve {
  // public Command trajectoryFollowerCommand(PathPlannerTrajectory trajectory) {
  //   Constants.AutoConstants.kPThetaController.enableContinuousInput(-Math.PI, Math.PI);
  //   return trajectoryFollowerCommand(
  //       trajectory,
  //       Constants.Drivetrain.kXController,
  //       Constants.Drivetrain.kYController,
  //       Constants.Drivetrain.kThetaController);
  // }

  static final MAXSwerveModule frontLeft =
      new MAXSwerveModule(Constants.SwerveDriveConstants.FrontLeftModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule frontRight =
      new MAXSwerveModule(Constants.SwerveDriveConstants.FrontRightModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule backLeft =
      new MAXSwerveModule(Constants.SwerveDriveConstants.BackLeftModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule backRight =
      new MAXSwerveModule(Constants.SwerveDriveConstants.BackRightModule.S_MODULE_CONSTANTS);

  public DriveSubsystem(AHRS m_gyro) {
    super(
        frontLeft,
        frontRight,
        backLeft,
        backRight,
        Constants.SwerveDriveConstants.kDriveKinematics,
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        m_gyro,
        Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond);
  }



  // TODO: Add sendable data for controllers during autonomous mode
  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   super.initSendable(builder);
  //   addChild("X Controller", Constants.SwerveConstants.kXController);
  //   addChild("Y Controller", Constants.SwerveConstants.kYController);
  //   addChild("Theta Controller", Constants.SwerveConstants.kThetaController);
  // }
}
