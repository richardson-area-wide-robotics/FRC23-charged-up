package frc.lib.swerve;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.localization.Localizer;

public class MAXSwerve extends SubsystemBase {
  public enum ModuleLocation {
    frontLeft(0),
    frontRight(1),
    backLeft(2),
    backRight(3);

    public final int value;
    private static final ModuleLocation[] m_mapping =
        new ModuleLocation[] {frontLeft, frontRight, backLeft, backRight};

    private ModuleLocation(int v) {
      this.value = v;
    }

    public static ModuleLocation fromInt(int v) {
      return m_mapping[v];
    }
  }
  // Robot MAXswerve modules
  private final MAXModule m_frontLeft;
  private final MAXModule m_frontRight;
  private final MAXModule m_backLeft;
  private final MAXModule m_backRight;
  private Localizer local;

  // The gyro sensor
  private final AHRS m_gyro;

  // The module positions
  private final SwerveModulePosition[] m_ModulePositions;

  // Kinematics
  private final SwerveDriveKinematics m_kinematics;

  // Odometry
  SwerveDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();

  private final double m_maxSpeed;

  /**
   * Create a Swerve Drive module
   *
   * @param frontLeft Swerve Module
   * @param frontRight Swerve Module
   * @param rearLeft Swerve Module
   * @param rearRight Swerve Module
   * @param kinematics Swerve drive kinematics
   * @param gyro used for odometry and field centric driving
   * @param maxSpeed of the wheels used to normalize wheel speeds
   */
  public MAXSwerve(
      MAXModule frontLeft,
      MAXModule frontRight,
      MAXModule backLeft,
      MAXModule backRight,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] modulePositions,
      AHRS gyro,
      double maxSpeed) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_backLeft = backLeft;
    m_backRight = backRight;
    m_gyro = gyro;
    m_kinematics = kinematics;
    m_ModulePositions = modulePositions;
    m_maxSpeed = maxSpeed;
    m_odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getAngle()), modulePositions);
    //m_odometry = new SwerveDriveOdometry(kinematics, m_gyro.getRotation2d(), modulePositions);
  }

  private ChassisSpeeds m_chassisSpeed = new ChassisSpeeds();

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity vx", () -> m_chassisSpeed.vxMetersPerSecond, null);
    builder.addDoubleProperty("Velocity vy", () -> m_chassisSpeed.vyMetersPerSecond, null);
    builder.addDoubleProperty("Velocity omega", () -> m_chassisSpeed.omegaRadiansPerSecond, null);
    addChild("Gyro", m_gyro);
    addChild("frontLeft", m_frontLeft);
    addChild("frontRight", m_frontRight);
    addChild("rearLeft", m_backLeft);
    addChild("rearRight", m_backRight);
    addChild("Field 2d", m_field);
  }

  @Override
  public void periodic() {
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_backLeft.periodic();
    m_backRight.periodic();

    m_chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());

    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  /**
   * Return the current velocity of the chassis as a ChassisSpeeds object.
   *
   * @return velocity of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeed;
  }

  /**
   * Predict the motion between the current position and a future state.
   *
   * @param lookahead Time in seconds to predict ahead.
   * @return twist2d represnting the change in pose over the lookahead time.
   */
  public Twist2d getPredictedMotion(double lookahead) {
    ChassisSpeeds chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());
    return new Twist2d(
        chassisSpeed.vxMetersPerSecond * lookahead,
        chassisSpeed.vyMetersPerSecond * lookahead,
        chassisSpeed.omegaRadiansPerSecond * lookahead);
  }

  /**
   * Return the modules positions
   *
   * @return SwerveModulePositions
   */
  public SwerveModulePosition[] getModulePositions() {
    return m_ModulePositions;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(Rotation2d.fromDegrees(getAngle()), getModulePositions(), pose);
    //m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // If nothing is commanded, hold the same position - don't know if the rev modules need this
    // yet?
    if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
      holdAllModulesRotation();
      return;
    }
    // Adjust input based on max speed
    xSpeed *= m_maxSpeed;
    ySpeed *= m_maxSpeed;
    rot *= Constants.SwerveDriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }


  /* These methods will prevent module movement if no command is being executed on the wheels */
  private void holdModuleRotation(MAXModule m) {
    var state = m.getDesiredState();
    state.speedMetersPerSecond = 0.0;
    m.setDesiredState(state);
  }

  private void holdAllModulesRotation() {
    holdModuleRotation(m_frontLeft);
    holdModuleRotation(m_frontRight);
    holdModuleRotation(m_backLeft);
    holdModuleRotation(m_backRight);
  }

  /** Set the swerve drive into an X which is not drivable but should help prevent being pushed. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Set an individual module state independently of all other modules. This should only be used for
   * testing/tuning.
   */
  public void testPeriodic() {
    m_frontLeft.testPeriodic();
    m_frontRight.testPeriodic();
    m_backLeft.testPeriodic();
    m_backRight.testPeriodic();
  }

  public SwerveModuleState[] getModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState(),
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** Calibrate the gyro. Requirements for the device being still depend on the gyro being used. */
  public void calibrateGyro() {
    m_gyro.calibrate();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
    // return m_gyro.getRotation2d().getDegrees();
  }

  public double getAdjustedAngle(){
    return 0.0;
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  public Command stopCommand() {
    return new RunCommand(this::stop, this).withName("Swerve Stop");
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.SwerveDriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getAngle(){
    return m_gyro.getAngle() * (Constants.SwerveDriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getRoll(){
    return Math.toRadians(-m_gyro.getRoll());
  }

  public void putNumber(){
    SmartDashboard.putNumber("Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Roll", -m_gyro.getRoll());
  }

  /**
   * Create a trajectory following command. Note that the beginning and end states of the command
   * are not necessarily 0 speed.
   *
   * @param trajectory PathPlanner trajectory
   * @param xController PID Controller for the X direction (left/right)
   * @param yController PID Contorller for the Y direction (forward/back)
   * @param thetaController Turning PID Controller for rotation (CCW Positive)
   * @return Command to be scheduled
   */
  public Command trajectoryFollowerCommand(
    PathPlannerTrajectory trajectory,
    PIDController xController,
    PIDController yController,
    PIDController thetaController) {
  Command swCommand =
      new PPSwerveControllerCommand(
          trajectory,
          this::getPose,
          m_kinematics,
          xController,
          yController,
          thetaController,
          (states) -> setModuleStates(states),
          true,
          this);
  return new InstantCommand(() -> m_field.getObject("Trajectory").setTrajectory(trajectory))
      .alongWith(swCommand);
}
}
