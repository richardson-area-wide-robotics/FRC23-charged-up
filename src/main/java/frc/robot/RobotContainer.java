// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FlashLeds;
import frc.robot.commands.LockMode.Lock;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.armPosition;
import frc.robot.subsystems.camera.Camera;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led_strip.LEDStrip;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.RoboState;
import frc.robot.subsystems.localization.Localizer;

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
  private Lock lockMode;
  private final Intake intake = new Intake();
  private final Camera camera = new Camera("Slotheye");
  private final  RoboState roboCon = new RoboState();
  private final Arm m_arm = new Arm();
  private final LEDStrip m_LEDStripLeft =
      new LEDStrip(LEDConstants.LED_STRIP_LEFT_PORT, LEDConstants.LED_STRIP_LEFT_LENGTH);
  private final LEDStrip m_LEDStripRight =
      new LEDStrip(LEDConstants.LED_STRIP_RIGHT_PORT, LEDConstants.LED_STRIP_RIGHT_LENGTH);
  private final LEDStrip[] m_LEDStrips = {m_LEDStripLeft, m_LEDStripRight};
  private final Localizer m_localizer;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Localizer localizer) {
     
    m_localizer = localizer;
   
   
    // Configure the trigger bindings
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


    //Some adjustments made for lock on mode
    DoubleSupplier moveForward =  () -> MathUtil.applyDeadband(
      -m_driverController.getLeftY(), 0.06); // 0.1 might be better?
     DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
      -m_driverController.getLeftX(), 0.06); // 0.1 might be better?
  
    lockMode = new Lock(m_robotDrive, camera, moveForward, moveSideways);

    //sends the movement information to RoboCon method in RoboState
    roboCon.drive(moveForward, moveSideways); 
    
    //Enters Lock-on mode
     new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(lockMode);
   
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

          //Make sure that all buttons are unique

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> intake.toggleIntake(), intake));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(
            new InstantCommand(
                () -> m_arm.moveArmToPosition(armPosition.INTAKE_ARM_POSITION_GROUND), m_arm));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(
            new InstantCommand(
                () -> m_arm.moveArmToPosition(armPosition.INTAKE_ARM_POSITION_STOWED), m_arm));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(
            new InstantCommand(
                () -> m_arm.moveArmToPosition(armPosition.SCORING_ARM_POSITION_MID), m_arm));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new SetLEDColor(m_LEDStrips, LEDConstants.YELLOW));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new FlashLeds(m_LEDStrips, LEDConstants.PURPLE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public class SetLEDColor extends CommandBase {
    private LEDStrip[] ledStrips;
    private int[] color;

    public SetLEDColor(LEDStrip[] ledStrips, int[] color) {
      this.ledStrips = ledStrips;
      this.color = color;
    }

    @Override
    public void initialize() {
      for (LEDStrip ledStrip : ledStrips) {
        ledStrip.setSolidColor(color);
      }
    }

    @Override
    public void end(boolean interrupted) {
      for (LEDStrip ledStrip : ledStrips) {
        ledStrip.setLightsToOff();
      }
    }
  }
}
