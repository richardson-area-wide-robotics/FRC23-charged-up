// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
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
import frc.lib.util.JoystickUtil;
import frc.robot.Constants.OIConstants;
import frc.robot.auton.paths.top.TopPark;
import frc.robot.auton.util.AutoChooser;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led_strip.LEDStrip;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.RoboState;
import frc.robot.subsystems.localization.Localizer;
import frc.robot.commands.SolidLeds;

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
  private final LEDStrip m_LEDStrip;
  // //private final LEDStrip m_LEDStripRight;
  // private final LEDStrip[] m_LEDStrips;

  {
    AutoChooser.setDefaultAuton(new TopPark(m_robotDrive));
  }

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final PositionCommand armPositions = new PositionCommand(m_arm, intake);


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     m_LEDStrip = 
      new LEDStrip(LEDConstants.LED_STRIP_PORT, LEDConstants.LED_STRIP_LENGTH);

    //  //m_LEDStripRight =
    //  // new LEDStrip(LEDConstants.LED_STRIP_RIGHT_PORT, LEDConstants.LED_STRIP_RIGHT_LENGTH);
    //   m_LEDStrips = new LEDStrip[] {m_LEDStripLeft};
   
    // Configure the trigger bindings
    configureDriverBindings(); 
    configureOperatorBindings();   
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

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> intake.manipulates(0.25)));
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

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .onTrue(armPositions.armPickUpTConeComand()).whileTrue(new RunCommand(()-> intake.manipulates(-1.0))).onFalse(armPositions.armStowCommand().alongWith(new RunCommand(()-> intake.manipulates(direction))));

    // Standing Cone 
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .onTrue(armPositions.armPickUpConeCommand()).whileTrue(new RunCommand(()-> intake.manipulates(-1.0))).onFalse(armPositions.armStowCommand()).whileFalse(new RunCommand(()->intake.manipulates(direction)));
    // Pick up Cube 
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .onTrue(armPositions.armPickUpCubeCommand()).whileTrue(new RunCommand(()-> intake.manipulates(1.0))).onFalse(armPositions.armStowCommand()).whileFalse(new RunCommand(()->intake.manipulates(direction)));
    // Shelf 
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
      .onTrue(armPositions.armPickUpFromShelf()).whileTrue(new RunCommand(()-> intake.manipulates(-1.0)).alongWith(new SolidLeds(m_LEDStrip, LEDConstants.YELLOW)));

    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value).whileTrue(new SolidLeds(m_LEDStrip, LEDConstants.PURPLE));
    // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    //     .whileTrue(new SetLEDColor(m_LEDStrips, LEDConstants.YELLOW));

    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    //     .whileTrue(new FlashLeds(m_LEDStrips, LEDConstants.PURPLE));
  }

  private void configureOperatorBindings() {

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

    // cone high
    // new JoystickButton(m_operatorController, XboxController.Button.kY.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(8)), new WaitCommand(0.3), new InstantCommand(() -> m_arm.moveElbowPosition(8))));
    new JoystickButton(m_operatorController, XboxController.Button.kY.value).onTrue(armPositions.armScoreConeHighCommand()).whileTrue(new SolidLeds(m_LEDStrip, LEDConstants.BLUE));
  // cone mid
    // new JoystickButton(m_operatorController, XboxController.Button.kB.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(9)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveElbowPosition(9))));
    new JoystickButton(m_operatorController, XboxController.Button.kB.value).onTrue(armPositions.armScoreConeMidCommand()).whileTrue(new SolidLeds(m_LEDStrip, LEDConstants.GREEN));
  // cube High
    // new JoystickButton(m_operatorController, XboxController.Button.kA.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(7)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveElbowPosition(7))));
    new JoystickButton(m_operatorController, XboxController.Button.kX.value).onTrue(armPositions.armScoreCubeHighCommand()).whileTrue(new SolidLeds(m_LEDStrip, LEDConstants.BLUE));
  // cube mid
    // new JoystickButton(m_operatorController, XboxController.Button.kX.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(6)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveElbowPosition(6))));
    new JoystickButton(m_operatorController, XboxController.Button.kA.value).onTrue(armPositions.armScoreCubeMidCommand()).whileTrue(new SolidLeds(m_LEDStrip, LEDConstants.GREEN));
    // shelf
    // new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(0)), new WaitCommand(0.7), new InstantCommand(() -> m_arm.moveArmToPosition(0))));

     /*
      * ---Manual arm controls 
      * Up on the dpad on the controller controls manual intake(elbow) up command
      * Down on the dpad on the controller controls manual intake(elbow) down command
      * left on the dpad on the controller controls manual intake(elbow) angle up command
      * right on the dpad on the controller controls manual intake(elbow) angle down command
      */
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

  public void autonInit() {
    m_robotDrive.calibrateGyro();
    m_robotDrive.stop();
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
