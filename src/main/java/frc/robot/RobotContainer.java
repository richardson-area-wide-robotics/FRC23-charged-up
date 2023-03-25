// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.auton.commands.BalancingCommand;
import frc.robot.auton.paths.top.TopMidScore2Park;
import frc.robot.auton.paths.top.TopMidScore3;
import frc.robot.auton.paths.bottom.BottomMidScore2;
import frc.robot.auton.paths.bottom.BottomMidScore2Park;
import frc.robot.auton.paths.bottom.BottomMidScore3;
import frc.robot.auton.paths.middle.MidScorePark;
import frc.robot.auton.paths.middle.MidScoreP1Park;
import frc.robot.auton.paths.top.TopMidScore2P1Park;
import frc.robot.auton.util.AutoChooser;
import frc.robot.auton.util.AutonUtil;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.localization.Localizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private double direction = 0.0;

  // The robot's subsystems
  private final AHRS m_gyro = new AHRS();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  private final Intake intake = new Intake();
  private final Arm m_arm = new Arm();
  private Localizer localizer;
  private final PositionCommand armPositions = new PositionCommand(m_arm);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  {
    /* Top Autonomous Routines */
    // new TopMidScore2P1Park(
    //   m_robotDrive, 
    //   intake, 
    //   m_arm);
    // new TopMidScore2Park(
    //   m_robotDrive, 
    //   m_arm, 
    //   intake);
    // new TopMidScore3(
    //   m_robotDrive, 
    //   intake,
    //   m_arm);
    // /* Middle Autonomous Routines */
    // new MidScoreP1Park(
    //   m_robotDrive, 
    //   intake, 
    //   m_arm);
    // new MidScorePark(
    //   m_robotDrive, 
    //   intake, 
    //   m_arm);
    // /* Bottom Autonomous Routines */
    // new BottomMidScore2(
    // m_robotDrive, 
    // intake, 
    // m_arm);
    // new BottomMidScore2Park(
    // m_robotDrive, 
    // intake, 
    // m_arm);
    // new BottomMidScore3(
    // m_robotDrive, 
    // intake, 
    // m_arm);
    AutoChooser.setDefaultAuton(new TopMidScore3(m_robotDrive, intake, m_arm));
  }
  
  // TODO: remove this before merging
  private BalancingCommand balance = new BalancingCommand(m_robotDrive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try{
      this.localizer = new Localizer("BACK");
      updateVisionPose().schedule();
    } catch (IOException e){
      e.printStackTrace();
      this.localizer = null;
    }
   
   
    // Configure the trigger bindings
    configureDriverBindings(); 
    configureOperatorBindings();   
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
  private void configureDriverBindings() {

    // TODO: remove this before merging
    //new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(balance);

    //Some adjustments made for lock on mode
    DoubleSupplier moveForward =  () -> MathUtil.applyDeadband(
      -m_driverController.getLeftY(), Constants.OIConstants.kControllerDeadband);
     DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
      -m_driverController.getLeftX(), Constants.OIConstants.kControllerDeadband);
  
    // lockMode = new Lock(m_robotDrive, camera, moveForward, moveSideways);

    //sends the movement information to RoboCon method in RoboState
    // roboCon.drive(moveForward, moveSideways); 
    
    //Enters Lock-on mode
    //  new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(lockMode);
    
    // Configure default commands
    /* 
       * ---Driving Controls for the driver 
       * The left stick on Xbox controller controls the translation of the robot - 1
       * The right stick controls the rotation of the robot - 12
       */
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () ->
          m_robotDrive.drive(
          moveForward.getAsDouble(),
          moveSideways.getAsDouble(),
          JoystickUtil.squareAxis(
          -m_driverController.getRightX()),
          true), m_robotDrive));

      /*
       * ---Reset button and X mode button
       * left stick button on controller controls the re-zeroing of the heading 
       * right stick button on controller controls the X mode of the robot
       */

       new JoystickButton(m_driverController, XboxController.Button.kRightStick.value).onTrue(new InstantCommand(()-> m_robotDrive.zeroHeading()));
       new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value).onTrue(new InstantCommand(()-> m_robotDrive.setX()));

    /*
     * ---Toggle buttons
     * Top Right botton on the back of Xbox controller controls the toggle between cone and intake state - P1
     */

    /*
     * ---Intaking Controls
     * Left trigger - intaking - 16
     * Right trigger - outaking - 13
     */
    // TODO: change this to be a ramp up with the deadband of the trigger :)

// if (new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).getAsBoolean()) {
//    if (mode) {
//       // Current state is true so turn off
//       mode = false;
//    } else {
//       // Current state is false so turn on
//       mode = true;
//    }
//   }

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(new RunCommand(()->intake.manipulates(-1)));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(new RunCommand(()->intake.manipulates(0.5)));

    /*
     * ---Arm Controls 
     * B - arm stowing
     * Y button - tipped cone pickup
     * X button - standing cone pickup
     * A button - cube picup
     * left Bumper - shelf cone pickup
     *  - shelf cube picup
     */
    if (m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_CONE || m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_TCONE || m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_SHELF){
      direction = -0.1;
    } else if (m_arm.getLastArmPosition() == ArmPositions.Positions.ARM_PICK_UP_CUBE){
      direction = 0.1;
      }

    // Stow
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(armPositions.armStowCommand());
    // Tipped pick up
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(armPositions.armPickUpTConeComand()).whileTrue(new RunCommand(()-> intake.manipulates(-1.0))).onFalse(armPositions.armStowCommand().alongWith(new RunCommand(()-> intake.manipulates(direction))));
    // Standing Cone 
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(armPositions.armPickUpConeCommand()).whileTrue(new RunCommand(()-> intake.manipulates(-1.0))).onFalse(armPositions.armStowCommand()).whileFalse(new RunCommand(()->intake.manipulates(direction)));
    // Pick up Cube 
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(armPositions.armPickUpCubeCommand()).whileTrue(new RunCommand(()-> intake.manipulates(0.5))).onFalse(armPositions.armStowCommand()).whileFalse(new RunCommand(()->intake.manipulates(direction)));
    // Shelf 
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value).onTrue(armPositions.armPickUpFromShelf()).whileTrue(new RunCommand(()-> intake.manipulates(-1.0)));
  }

  private void configureOperatorBindings(){
    /* 
     * ---Toggle button that switches the state of the robot - between cone or cube state
     * left trigger on controller controls the toggle between cone and intake state - P1
    */

    /*
     * ---Arm Controls
     * Y button on the controller controls High cone/cube score
     * A button on the controller controls Mid cone/cube pickup
     * B button on the controller controls Low cone/cube pickup
     * X button on the controller controls Stow
     */

    // cone high
    new JoystickButton(m_operatorController, XboxController.Button.kY.value).onTrue(armPositions.autonArmScoreConeHighCommand());
    // cone mid
    new JoystickButton(m_operatorController, XboxController.Button.kB.value).onTrue(armPositions.armScoreConeMidCommand());
    // cube High
    new JoystickButton(m_operatorController, XboxController.Button.kX.value).onTrue(armPositions.armScoreCubeHighCommand());
    // cube mid
    new JoystickButton(m_operatorController, XboxController.Button.kA.value).onTrue(armPositions.armScoreCubeMidCommand());
    // shelf
    
  }

  /**
   * Sets the idle mode for the arm and intake joints
   * 
   * Used to make the robot arm easier to move when disabled
   */
  public void setIdleMode(IdleMode idleMode){
      m_arm.getSparkStatus(idleMode);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    System.out.println("Auton Selected" + AutoChooser.getAuton().getName());
    return AutoChooser.getAuton();
  }

  /**
   * Use this method to pass anythign to the dashboard 
   * 
   * Reduces multi method use to Shuffleboard
   */
  public void putDashboard(){
    SmartDashboard.putNumber("Gyro Angle", m_robotDrive.getAngle());
    SmartDashboard.putNumber("Arm encoder", m_arm.getArmAbsoluteEncoder());
    SmartDashboard.putNumber("encoder elbow", m_arm.getElbowAbsoluteEncoder());
    m_robotDrive.putNumber();
  }

  /** Run a function at the start of auton. */
  public void autonInit(){
    m_robotDrive.calibrateGyro();
    m_robotDrive.stop();
    this.globalEventList();
  }

  /** Creates the Global event list for the autonomous paths */
  public void globalEventList(){
    AutonUtil.addEvent("IntakeDownCone", armPositions.armPickUpTConeComand());
    AutonUtil.addEvent("ScoreCone", armPositions.armScoreConeMidCommand());
    AutonUtil.addEvent("IntakeDown", armPositions.armPickUpCubeCommand());
    AutonUtil.addEvent("Stow", armPositions.armStowCommand());
    AutonUtil.addEvent("Score", armPositions.armScoreCubeMidCommand());
  }

  /** Run a function during autonomous to get run time of autonomous. */
  public void autonPeriodic(){
    SmartDashboard.putNumber("Auton Time", Timer.getFPGATimestamp());
  }

  public Command updateVisionPose(){
    return new RunCommand(() -> {
      Optional<Pose3d> pose = localizer.getRobotPose();
      // more lines here as needed
      if (!pose.isEmpty()){
        m_robotDrive.addPoseEstimate(
        pose.get().toPose2d(),
        localizer.getPoseTimeStamp().get());
      }
    });
  }
}
