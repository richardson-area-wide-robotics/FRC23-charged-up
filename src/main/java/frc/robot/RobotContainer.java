// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.JoystickUtil;
import frc.robot.Constants.OIConstants;
import frc.robot.auton.commands.Balance;
import frc.robot.auton.paths.top.TopPark;
import frc.robot.auton.util.AutoChooser;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private double direction = 0.0;
  // The robot's subsystems
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  private final Intake intake = new Intake();
  private final Arm m_arm = new Arm();

  {
    AutoChooser.setDefaultAuton(new TopPark(m_robotDrive));
  }

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final PositionCommand armPositions = new PositionCommand(m_arm, intake);
  private Balance balance = new Balance(m_robotDrive);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureDriverBindings();
    configureOpperatorBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(balance);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        /*
         * ---Driving Controls for the driver
         * The left stick on Xbox controller controls the translation of the robot - 1
         * The right stick controls the rotation of the robot - 12
         */

        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(),
                    Constants.OIConstants.kControllerDeadband),
                MathUtil.applyDeadband(-m_driverController.getLeftX(),
                    Constants.OIConstants.kControllerDeadband),
                JoystickUtil.squareAxis(
                    -m_driverController.getRightX()),
                true),
            m_robotDrive));

    /*
     * ---Reset button and X mode button
     * left stick button on controller controls the re-zeroing of the heading
     * right stick button on controller controls the X mode of the robot
     */

    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setX()));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> intake.manipulates(-1)));

    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    //     .whileTrue(new RunCommand(() -> intake.manipulates(0.25)));
    if (m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_CONE
        || m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_TCONE
        || m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_SHELF) {
      direction = -0.1;
    } else if (m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_CUBE) {
      direction = 0.1;
    }

    // Stow
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(armPositions.armStowCommand());
    // Tipped pick up

    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(armPositions.armPickUpTConeComand())
        .whileTrue(new RunCommand(() -> intake.manipulates(-1.0)))
        .onFalse(armPositions.armStowCommand().alongWith(new RunCommand(() -> intake.manipulates(direction))));

    // Standing Cone
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(armPositions.armPickUpConeCommand())
        .whileTrue(new RunCommand(() -> intake.manipulates(-1.0))).onFalse(armPositions.armStowCommand())
        .whileFalse(new RunCommand(() -> intake.manipulates(direction)));
    // Pick up Cube
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(armPositions.armPickUpCubeCommand())
        .whileTrue(new RunCommand(() -> intake.manipulates(1.0))).onFalse(armPositions.armStowCommand())
        .whileFalse(new RunCommand(() -> intake.manipulates(direction)));
    // Shelf
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .onTrue(armPositions.armPickUpFromShelf()).whileTrue(new RunCommand(() -> intake.manipulates(-1.0)));
  }

  private void configureOpperatorBindings() {

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .onTrue(armPositions.armScoreConeHighCommand());
    // cone mid
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .onTrue(armPositions.armScoreConeMidCommand());
    // cube High
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .onTrue(armPositions.armScoreCubeHighCommand());
    // cube mid
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .onTrue(armPositions.armScoreCubeMidCommand());

  }

  public double getSparkMax() {
    return m_arm.getArmAbsoluteEncoder();
  }

  public double getElbowSparkMax() {
    return m_arm.getElbowAbsoluteEncoder();
  }

  public void getIdleMode(IdleMode idleMode) {
    m_arm.getSparkStatus(idleMode);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return AutoChooser.getAuton();
  }

  public void putAccel(){
  SmartDashboard.putNumber("Gyro Accel X", m_gyro.getAccelX());
  SmartDashboard.putNumber("Gyro Accel y", m_gyro.getAccelY());
  SmartDashboard.putNumber("Gyro Accel z", m_gyro.getAccelZ());
  }

  public void autonInit() {
    m_robotDrive.calibrateGyro();
    m_robotDrive.stop();
  }

  public void getPitch(){
    SmartDashboard.putNumber("Pitch",balance.getPitch());
  }
}
