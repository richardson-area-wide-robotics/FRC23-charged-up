package frc.robot.auton.paths.top;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.util.AutonBase;
import frc.robot.auton.util.AutonUtil;
import frc.robot.commands.armCommands.ElbowPosition;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.commands.armCommands.ShoulderPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

public class Top3 extends AutonBase {
    public PositionCommand armPositions;
    public ElbowPosition elbowCommand;
    public ShoulderPosition shoulderPosition;
    public Arm arm;

    public Top3(
    DriveSubsystem drive, 
    Intake intake,
    Arm m_arm) {
      
    List<PathPlannerTrajectory> topLink = AutonUtil.loadTrajectoryGroup("Top-Three", new PathConstraints(2.5, 3.0), new PathConstraints(4.0, 5.0), new PathConstraints(4.0, 5.0));
    PathPlannerTrajectory firstPath = topLink.get(0);
    PathPlannerTrajectory secondPath = topLink.get(1);
    PathPlannerTrajectory thirdPath = topLink.get(2);

    Pose2d initialPose = AutonUtil.initialPose(firstPath);
    this.armPositions = new PositionCommand(m_arm);
    AutonUtil.addEvent("SpecialIntakeDown", specialArmPickUpCubeCommand());
    AutonUtil.addEvent("SpecialScoreCube", specialArmScoreCubeCommand());
    AutonUtil.addEvent("SpecialScoreCubeHigh", specialArmScoreCubeHighCommand());
    AutonUtil.addEvent("IntakeDownFromHigh", IntakeDownFromHigh());

    /*
     * Checks if the paths are null and if they are it will print out that the path was not found
     */
    if (firstPath == null && secondPath == null && thirdPath == null) {
        System.out.println("Path not found");
        return;
    }

     /**
     * Creates a command group that this auton class will call on when initialized 
     */
    addCommandsWithLog("Top Three",
      /* Runs commands to score pre-load Cone */
      new RunCommand(()-> intake.setIntakeSpeed(1.0), intake)
        .raceWith(armPositions.armScoreConeHighCommand())
          .andThen(new WaitCommand(0.7))
            .andThen(new RunCommand(()-> intake.setIntakeSpeed(-0.25), intake).withTimeout(0.7))
              .andThen(armPositions.armStowCommand())
      // /* 
      //  * Resets the Odometry of the drivetrain to the starting pose of the first path 
      //  */
      .andThen(new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"))

      // /* 
      //  * Runs the First path which is picking up the first cube 
      //  * and races with the intake to pick up the cube
      //  */
      .andThen(new RunCommand(()-> intake.setIntakeSpeed(-1.0), intake)
        .raceWith(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(firstPath), firstPath)))
      .andThen(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(secondPath), secondPath))

      //  /*
      //  * Activate intake to score the first cube
      //  * and then stop the intake 
      //  */
      .andThen(new WaitCommand(0.2))
        .andThen(new RunCommand(()-> intake.setIntakeSpeed(1.0), intake).withTimeout(0.3))

      // // /*
      // //  * Runs the Second path which is to pick up a cone
      // //  */
      .andThen(new RunCommand(()-> intake.setIntakeSpeed(1.0), intake)
        .raceWith(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(thirdPath), thirdPath)))

      // // /*
      // //  * Activate intake to score the cone
      // //  */
      .andThen(new RunCommand(()-> intake.setIntakeSpeed(-1.0), intake)));
    }

  public Command specialArmPickUpCubeCommand(){
  return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_MID)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
  }

  public Command specialArmScoreCubeHighCommand(){
  return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_MID - 0.025)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
  }

  public Command specialArmScoreCubeCommand(){
  return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_MID - 0.025)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
  }

  public Command IntakeDownFromHigh(){
    return new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CUBE));//new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_MID - 0.025)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
