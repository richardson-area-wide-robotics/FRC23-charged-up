// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.auton.paths.top.TopPark;
import frc.robot.auton.util.AutoChooser;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.lockMode.Lock;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.camera.Camera;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.RoboState;
import frc.robot.subsystems.localization.Localizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private boolean coneState = false; 
  // The robot's subsystems
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  //private final Intake intake = new Intake();
  //private final Arm m_arm = new Arm();

  {
    AutoChooser.setDefaultAuton( new TopPark(m_robotDrive));
  }
  
  private Lock lockMode;
  private final Intake intake = new Intake();
  // private final Camera camera = new Camera("Slotheye");
  // private final  RoboState roboCon = new RoboState();
  private final Arm m_arm = new Arm();
  // private final Localizer m_localizer;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private IntakeCommand intakeCommand;

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     
    // m_localizer = localizer;
   
   
    // Configure the trigger bindings
    configureDriverBindings(); 
    configureOpperatorBindings();   
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

    

    //Some adjustments made for lock on mode
    DoubleSupplier moveForward =  () -> MathUtil.applyDeadband(
      -m_driverController.getLeftY(), 0.06); // 0.1 might be better?
     DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
      -m_driverController.getLeftX(), 0.06); // 0.1 might be better?
  
    // lockMode = new Lock(m_robotDrive, camera, moveForward, moveSideways);

    //sends the movement information to RoboCon method in RoboState
    // roboCon.drive(moveForward, moveSideways); 
    
    //Enters Lock-on mode
    //  new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(lockMode);
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      /* 
       * ---Driving Controls for the driver 
       * The left stick on Xbox controller controls the translation of the robot - 1
       * The right stick controls the rotation of the robot - 12
       */
  
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

      /*
       * ---Reset button and X mode button
       * left stick button on controller controls the re-zeroing of the heading 
       * right stick button on controller controls the X mode of the robot
       */

          if(m_driverController.getRightStickButtonPressed()){
            m_robotDrive.setX();
          }
    if(m_driverController.getRightStickButtonPressed()){
      m_robotDrive.setX();
    }

          //Make sure that all buttons are unique

    /*
     * ---Toggle buttons
     * Top Right botton on the back of Xbox controller controls the toggle between cone and intake state - P1
     */

    //  new JoystickButton(m_driverController, XboxController.Button.kBack.value).whileTrue(new InstantCommand(() -> intake.intake(), intake));

    /*
     * ---Intaking Controls
     * Left trigger - intaking - 16
     * Right trigger - outaking - 13
     */
    // TODO: change this to be a ramp up with the deadband of the trigger :)

    boolean mode = m_driverController.getLeftStickButton();


    if (!mode){
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(new RunCommand(() -> intake.intake(1.0), intake));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(new RunCommand(() -> intake.outake(-1.0), intake));
    } else {
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(new RunCommand(() -> intake.intake(-1.0), intake));

      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(new RunCommand(() -> intake.outake(1.0), intake));
    }

    /*
     * ---Arm Controls 
     * Top back left button on the back - arm stowing
     * Y button - shelf cone pickup
     * A button - tipped cone pickup
     * B button - standing cone pickup
     * left bumper - shelf cube picup
     * Right bumper - cube picup
     */


    // pick up tipped cone
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> System.out.println("Button Presseds")), new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.2), new InstantCommand(() -> m_arm.moveArmToPosition(1)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveElbowPosition(1))));
    // stowed
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.7), new InstantCommand(() -> m_arm.moveArmToPosition(10)), new WaitCommand(1.6), new InstantCommand(() -> m_arm.moveElbowPosition(10))));
    //  pick up cube
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(3)), new WaitCommand(0.3), new InstantCommand(() -> m_arm.moveElbowPosition(3))));
    // standing cone 
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.7), new InstantCommand(() -> m_arm.moveArmToPosition(2)), new WaitCommand(1.0), new InstantCommand(() -> m_arm.moveElbowPosition(2))));
    // shelf

  }

  private void configureOpperatorBindings(){
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
    new JoystickButton(m_operatorController, XboxController.Button.kY.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(8)), new WaitCommand(0.3), new InstantCommand(() -> m_arm.moveElbowPosition(8))));
  // cube high
    new JoystickButton(m_operatorController, XboxController.Button.kB.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(9)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveElbowPosition(9))));
// cube mid
    new JoystickButton(m_operatorController, XboxController.Button.kA.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(7)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveElbowPosition(7))));
// cone mid
    new JoystickButton(m_operatorController, XboxController.Button.kX.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(11)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveArmToPosition(6)), new WaitCommand(0.5), new InstantCommand(() -> m_arm.moveElbowPosition(6))));
    // shelf
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value).onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_arm.moveElbowPosition(0)), new WaitCommand(0.7), new InstantCommand(() -> m_arm.moveArmToPosition(0))));
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));


     /*
      * ---Manual arm controls 
      * Up on the dpad on the controller controls manual intake(elbow) up command
      * Down on the dpad on the controller controls manual intake(elbow) down command
      * left on the dpad on the controller controls manual intake(elbow) angle up command
      * right on the dpad on the controller controls manual intake(elbow) angle down command
      */
  }

  public double getSparkMax(){
    // SparkMaxAbsoluteEncoder abs = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // return (abs.getPosition() * 2 * Math.PI) /*- Units.degreesToRadians()*/;
    return m_arm.getArmAbsoluteEncoder();
  }

  public double getElbowSparkMax(){
    // SparkMaxAbsoluteEncoder abs = ElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // return abs.getPosition() * 2 * Math.PI;
    return m_arm.getElbowAbsoluteEncoder();
  }

  // public double getnormalSparkMax(){
  //   RelativeEncoder abs = leftArmMotor.getEncoder();
  //   return abs.getPosition() /*- Units.degreesToRadians()*/;
  // }

  // public double getnormalElbowSparkMax(){
  //   RelativeEncoder abs = ElbowMotor.getEncoder();
  //   return abs.getPosition();
  // }

  public void getIdleMode(IdleMode idleMode){
      m_arm.getSparkStatus(idleMode);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    // AutoChooser.addAuton(, "Top-Park");
    return AutoChooser.getAuton();
  }

  public void autonInit(){
    m_robotDrive.calibrateGyro();
    m_robotDrive.stop();
  }
}
