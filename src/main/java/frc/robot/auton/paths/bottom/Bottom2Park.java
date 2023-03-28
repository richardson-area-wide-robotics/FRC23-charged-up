package frc.robot.auton.paths.bottom;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.commands.BalancingCommand;
import frc.robot.auton.util.AutonBase;
import frc.robot.auton.util.AutonUtil;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

public class Bottom2Park extends AutonBase {
    public PositionCommand armPositions;
    public BalancingCommand balance;
    
    public Bottom2Park(
    DriveSubsystem drive, 
    Intake intake,
    Arm m_arm){

    List<PathPlannerTrajectory> bottom2Park = AutonUtil.loadTrajectoryGroup("Bottom-Score-2Park", new PathConstraints(3.5, 4.0), new PathConstraints(2.0, 3.0), new PathConstraints(3.0, 4.0));
    PathPlannerTrajectory firstPath = bottom2Park.get(0);
    PathPlannerTrajectory secondPath = bottom2Park.get(1);
    PathPlannerTrajectory thirdPath = bottom2Park.get(2);

    Pose2d initialPose = AutonUtil.initialPose(firstPath);
    this.armPositions = new PositionCommand(m_arm);
    this.balance = new BalancingCommand(drive);

     /*
     * Checks if the paths are null and if they are it will print out that the path was not found
     */
    if (firstPath == null && secondPath == null) {
        System.out.println("Path not found");
        return;
    }

    /**
     * Creates a command group that this auton class will call on when initialized 
     */
    addCommandsWithLog("Top 2P1 Park",
      /* Runs commands to score pre-load Cone */
      new RunCommand(()-> intake.manipulates(-1.0), intake)
      .raceWith(armPositions.armScoreConeMidCommand())
        .andThen(new WaitCommand(0.5))
          .andThen(new RunCommand(()-> intake.manipulates(0.25), intake).withTimeout(0.5))

      /* 
       * Resets the Odometry of the drivetrain to the starting pose of the first path 
       */
      .andThen(new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"))

       /* 
       * Runs the First path which is picking up the first cube 
       * and races with the intake to pick up the cube
       */
      .andThen(new RunCommand(()-> intake.manipulates(1.0), intake)
        .raceWith(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(firstPath), firstPath)))
        .andThen(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(secondPath), secondPath))
        .andThen(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(thirdPath), thirdPath))
        // .raceWith(new FollowPathWithEvents(drive.trajectoryFollowerCommand(pathGroup.get(0)), pathGroup.get(0).getMarkers(), AutonUtil.getEventMap())))
      
       /*
       * Activate intake to score the first cube
       * and then stop the intake 
       */
      .andThen(new RunCommand(()-> intake.manipulates(-1.0), intake).withTimeout(0.3))
        .andThen(new RunCommand(()-> intake.manipulates(1.0), intake)));

      // //  /*
      // //  * Runs the Second path which is to pick up second cube and balance with it 
      // //  * and then activates balancing command
      // //  */
      // .raceWith(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(secondPath), secondPath)))
      //   .andThen(balance));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

 }


