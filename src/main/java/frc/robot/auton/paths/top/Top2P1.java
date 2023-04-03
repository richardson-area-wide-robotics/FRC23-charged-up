package frc.robot.auton.paths.top;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
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

public class Top2P1 extends AutonBase {
    public PositionCommand armPositions;
    public PathPlannerServer server = new PathPlannerServer();

    public Top2P1(
    DriveSubsystem drive, 
    Intake intake,
    Arm m_arm) {
      
    List<PathPlannerTrajectory> top2P1Park = AutonUtil.loadTrajectoryGroup("Top-Score-2P1Park", new PathConstraints(3.85, 5.0), new PathConstraints(3.0, 4.5));
    PathPlannerTrajectory firstPath = top2P1Park.get(0);
    PathPlannerTrajectory secondPath = top2P1Park.get(1);
    
    Pose2d initialPose = AutonUtil.initialPose(firstPath);
    this.armPositions = new PositionCommand(m_arm);
   BalancingCommand balancingCommand = new BalancingCommand(drive);

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
      new RunCommand(()-> intake.manipulates(1.0), intake)
      .raceWith(armPositions.armScoreConeMidCommand())
        .andThen(new WaitCommand(0.6))
          .andThen(new RunCommand(()-> intake.manipulates(-0.25), intake).withTimeout(0.5))

      /* 
       * Resets the Odometry of the drivetrain to the starting pose of the first path 
       */
      .andThen(new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"))

       /* 
       * Runs the First path which is picking up the first cube 
       * and races with the intake to pick up the cube
       */
      .andThen(new RunCommand(()-> intake.manipulates(-0.9), intake)
        .raceWith(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(firstPath), firstPath)))
        .andThen(new WaitCommand(0.3))
      
       /*
       * Activate intake to score the first cube
       * and then stop the intake 
       */
      .andThen(new RunCommand(()-> intake.manipulates(1.0), intake).withTimeout(0.6))
        .andThen(new RunCommand(()-> intake.manipulates(-1.0), intake)

      //  /*
      //  * Runs the Second path which is to pick up second cube and balance with it 
      //  * and then activates balancing command
      //  */
      .raceWith(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(secondPath), secondPath)))
      .andThen(armPositions.armStowCommand()));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
