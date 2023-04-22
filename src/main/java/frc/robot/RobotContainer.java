// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANSparkMax.IdleMode;

import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.auton.paths.top.Top3;
import frc.robot.auton.paths.top.TopLink;
import frc.robot.auton.paths.PathTester;
import frc.robot.auton.paths.bottom.Bottom2P1;
import frc.robot.auton.paths.middle.MidScorePark;
import frc.robot.auton.paths.top.Top2P1;
import frc.robot.auton.paths.top.Top2P1Park;
import frc.robot.auton.util.AutoChooser;
import frc.robot.auton.util.AutonUtil;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsController;
import frc.robot.subsystems.localization.Localizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final AHRS m_gyro = new AHRS();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  private final Intake intake = new Intake();
  private final Arm m_arm = new Arm();
  private Localizer frontLocalizer;
  private Localizer backLocalizer;
  public Runnable updateBackVisionPose;
  public Runnable updateFrontVisionPose;
  private LightsController lights;
  private Lights m_lights;
  private final PositionCommand armPositions = new PositionCommand(m_arm);
  // private ArmPositions positions = new ArmPositions();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverControllerSP = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  {
    /* Top Autonomous Routines */
    new Top2P1Park(
      m_robotDrive, 
      intake, 
      m_arm);
    new Top2P1(
      m_robotDrive, 
      intake, 
      m_arm);
    // new Top3(
    //   m_robotDrive, 
    //   intake, 
    //   m_arm);
    // /* Middle Autonomous Routines */
    // new MidScoreP1Park(
    //   m_robotDrive, 
    //   intake, 
    //   m_arm);
    new MidScorePark(
      m_robotDrive, 
      intake, 
      m_arm);
    // /* Bottom Autonomous Routines */
    // new BottomMidScore2(
    // m_robotDrive, 
    // intake, 
    // m_arm);
    new Bottom2P1(
    m_robotDrive, 
    intake, 
    m_arm);
    AutoChooser.setDefaultAuton(new PathTester(m_robotDrive));
  }
  
  // TODO: remove this before merging
  private BalancingCommand balance = new BalancingCommand(m_robotDrive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try{
      this.backLocalizer = new Localizer("BACK", Constants.Localizer.BACK_CAMERA_TO_ROBOT);
      this.updateBackVisionPose = updateVisionPose(backLocalizer);
    } catch (IOException e){
      e.printStackTrace();
      this.backLocalizer = null;
    }

    try{
      this.frontLocalizer = new Localizer("FRONT", Constants.Localizer.FRONT_CAMERA_TO_ROBOT);
      this.updateFrontVisionPose = updateVisionPose(frontLocalizer);
    } catch (IOException e){
      e.printStackTrace();
      this.frontLocalizer = null;
    }

    m_lights = new Lights(9, 100, 50);
    m_lights.allBlue();
   
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
    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(balance);

    // set up for driving controls
    DoubleSupplier moveForward =  () -> MathUtil.applyDeadband(
      -m_driverController.getLeftY(), Constants.OIConstants.kControllerDeadband);
     DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
      -m_driverController.getLeftX(), Constants.OIConstants.kControllerDeadband);
    
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

       m_driverController.rightStick().onTrue(new InstantCommand(()-> m_robotDrive.zeroHeading()));
       m_driverController.leftStick().onTrue(new InstantCommand(()-> m_robotDrive.setX()));

    /*
     * ---Intaking Controls
     * Left trigger - intaking - 16
     * Right trigger - outaking - 13
     * Up on dpad on the back of Xbox controller controls the toggle between cone and intake state using Intake - P1
     */

    intake.setDefaultCommand(intake.idle());
    m_driverController.leftTrigger().whileTrue(intake.manipulatorCommand(1.0));

    m_driverController.rightTrigger().whileTrue(intake.manipulatorCommand(-1.0));

    /*
     * ---Arm Controls 
     * B - arm stowing
     * Y button - tipped cone pickup
     * X button - standing cone pickup
     * A button - cube picup
     * left Bumper - shelf cone pickup
     *  - shelf cube picup
     */

    // Stow
    m_driverController.b().onTrue(armPositions.armStowCommand());

    // Tipped pick up
    m_driverController.y().onTrue(armPositions.armPickUpTConeComand()).onTrue(new InstantCommand(()->{ intake.setMode(this.intake.mode = false); m_lights.allYellow();}));
    // new JoystickButton(m_driverControllerSP, XboxController.Button.kY.value).onTrue(new InstantCommand(()-> intake.setXMode(false)));

    // Standing Cone 
    m_driverController.a().onTrue(armPositions.armBackStandingCone()).onTrue(new InstantCommand(()-> intake.setMode(this.intake.mode = false)));

    // Pick up Cube 
    m_driverController.x().onTrue(armPositions.armPickUpCubeCommand()).onTrue(new InstantCommand(()->{ intake.setMode(this.intake.mode = true); m_lights.allPurple();})).whileTrue(intake.manipulatorCommand(-1.0));
    // new JoystickButton(m_driverControllerSP, XboxController.Button.kX.value).onTrue(new InstantCommand(()-> intake.setXMode(true)));

    // Shelf single
    m_driverController.leftBumper().onTrue(armPositions.armPickUpFromShelf()).onTrue(new InstantCommand(()->{ intake.setMode(this.intake.mode = false); m_lights.allYellow();})).whileTrue(intake.manipulatorCommand(1.0));

    // // Shelf double
    // m_driverController.rightBumper().onTrue(armPositions.armPickUpFromDoubleShelf());//.onTrue(new InstantCommand(()-> intake.setMode(false))).whileTrue(intake.manipulatorCommand(1.0)).onTrue(new InstantCommand(()->m_arm.setAdjusted(1)));

    m_driverController.pov(0).onTrue(new InstantCommand(()->{intake.setMode(this.intake.mode = false); m_lights.allYellow();}));
    m_driverController.pov(180).onTrue(new InstantCommand(()->{intake.setMode(this.intake.mode = true); m_lights.allPurple();}));
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
    m_operatorController.y().onTrue(armPositions.armScoreConeHighCommand()).onTrue(new InstantCommand(()->m_arm.setAdjusted(1)));
    // cone mid
    m_operatorController.b().onTrue(armPositions.armScoreConeMidCommand()).onTrue(new InstantCommand(()->m_arm.setAdjusted(0)));
    // cube High
    m_operatorController.x().onTrue(armPositions.armScoreCubeHighCommand()).onTrue(new InstantCommand(()->m_arm.setAdjusted(0)));
    // cube mid
    m_operatorController.a().onTrue(armPositions.armScoreCubeMidCommand()).onTrue(new InstantCommand(()->m_arm.setAdjusted(0)));
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
    m_robotDrive.putNumber();
    SmartDashboard.putNumber("filtered PoseX", m_robotDrive.getPose().getX());
    SmartDashboard.putNumber("filtered PoseY", m_robotDrive.getPose().getY());
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
    AutonUtil.addEvent("ScoreCone", armPositions.autonArmScoreConeMidCommand());
    AutonUtil.addEvent("IntakeDown", armPositions.autonArmPickUpCubeCommand());
    AutonUtil.addEvent("Stow", armPositions.armStowCommand());
    AutonUtil.addEvent("Score", armPositions.armScoreCubeMidCommand());
    AutonUtil.addEvent("ScoreHighCone", armPositions.armScoreConeHighCommand());
  }

  /** Run a function during autonomous to get run time of autonomous. */
  public void autonPeriodic(){
    SmartDashboard.putNumber("Auton Time", Timer.getFPGATimestamp());

  }

  public Runnable updateVisionPose(Localizer localizer){
    return () -> {
      Optional<Pose3d> pose = localizer.getRobotPose();
      // more lines here as needed
      if (!pose.isEmpty()){
        Pose2d localizedPosed = new Pose2d(pose.get().toPose2d().getX(), pose.get().toPose2d().getY(), new Rotation2d(m_robotDrive.getAngle()));
        double distance = Math.hypot((m_robotDrive.getPose().getX() - pose.get().getX()), (m_robotDrive.getPose().getY() - pose.get().getY()));
        if (distance < Constants.Localizer.distanceOffset){
        m_robotDrive.addPoseEstimate(
        localizedPosed,
        localizer.getPoseTimeStamp().get());
        }
        SmartDashboard.putBoolean("using AprilTag", true);
      }
      else{

        SmartDashboard.putBoolean("using AprilTag", false);
      }
    };
  }
}
