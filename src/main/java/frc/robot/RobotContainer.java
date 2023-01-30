// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LockMode.Lock;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.armPosition;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  private final Lock LockMode;
  
  private final Intake intake = new Intake();
  private final Arm m_arm = new Arm();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  
    //Some adjustments made for lock on mode
     DoubleSupplier moveForward =  () -> MathUtil.applyDeadband(
      -m_driverController.getLeftY(), 0.06); // 0.1 might be better?
     DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
      -m_driverController.getLeftX(), 0.06); // 0.1 might be better?
      
    LockMode = new Lock(m_robotDrive, moveForward, moveSideways);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    moveForward.getAsDouble(), // 0.1 might be better?
                    moveSideways.getAsDouble(), // 0.1 might be better?
                    MathUtil.applyDeadband(
                        -m_driverController.getRightX(), 0.06), // 0.1 might be better?
                    true),
            m_robotDrive));
    configureBindings();    
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    new JoystickButton(m_driverController, XboxController.Button.kY.value).toggleOnTrue(LockMode);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    MathUtil.applyDeadband(
                        -m_driverController.getLeftY(), 0.1),
                    MathUtil.applyDeadband(
                        -m_driverController.getLeftX(), 0.1),
                    MathUtil.applyDeadband(
                        -m_driverController.getRightX(), 0.1),
                    true),
            m_robotDrive));

          if(m_driverController.getLeftStickButtonPressed()){
            m_robotDrive.zeroHeading();
          }

          if(m_driverController.getRightStickButtonPressed()){
            m_robotDrive.setX();
          }

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> intake.toggleIntake(), intake));

    new JoystickButton(m_driverController, XboxController.Button.kY.value).whileTrue(new InstantCommand(() -> m_arm.moveArmToPosition(armPosition.INTAKE_ARM_POSITION_GROUND), m_arm));

    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new InstantCommand(() -> m_arm.moveArmToPosition(armPosition.INTAKE_ARM_POSITION_STOWED), m_arm));

    new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(new InstantCommand(() -> m_arm.moveArmToPosition(armPosition.SCORING_ARM_POSITION_MID), m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
