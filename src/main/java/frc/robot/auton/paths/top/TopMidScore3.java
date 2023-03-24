package frc.robot.auton.paths.top;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
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

public class TopMidScore3 extends AutonBase {
    public PositionCommand armPositions;
    public BalancingCommand balance;
    public PathPlannerServer server = new PathPlannerServer();

    public TopMidScore3(
    DriveSubsystem drive, 
    Intake intake,
    Arm m_arm) {
      
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Top-Score-3", new PathConstraints(2.75, 4.5), new PathConstraints(3.0, 5.0));
    HashMap<String, Command> eventMap = new HashMap<>();

    Pose2d initialPose = AutonUtil.initialPose(pathGroup.get(0));
    this.armPositions = new PositionCommand(m_arm);
    this.balance = new BalancingCommand(drive);
  
    eventMap.put("IntakeDownCone", armPositions.armPickUpTConeComand());
    eventMap.put("ScoreCone", armPositions.armScoreConeMidCommand());
    eventMap.put("IntakeDown", armPositions.armPickUpCubeCommand());
    eventMap.put("Stow", armPositions.armStowCommand());
    eventMap.put("Score", armPositions.armScoreCubeMidCommand());

    if (pathGroup.get(0) == null && pathGroup.get(1) == null) {
        System.out.println("Path not found");
        return;
    }

    addCommandsWithLog("Top Link Auto",
      new RunCommand(()-> intake.manipulates(-1.0), intake)
      .raceWith(armPositions.armScoreConeMidCommand())
      .andThen(new WaitCommand(0.6))
      .andThen(new RunCommand(()-> intake.manipulates(0.25), intake).withTimeout(0.7))
      .andThen(new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"))
      .andThen(new RunCommand(()-> intake.manipulates(-1.0), intake)
      .raceWith(new FollowPathWithEvents(drive.trajectoryFollowerCommand(pathGroup.get(0)), pathGroup.get(0).getMarkers(), eventMap)))
      .andThen(new WaitCommand(0.8))
      .andThen(new RunCommand(()-> intake.manipulates(1.0), intake).withTimeout(0.5))
      .andThen(new RunCommand(()-> intake.manipulates(1.0), intake)
      .raceWith(new FollowPathWithEvents(drive.trajectoryFollowerCommand(pathGroup.get(1)), pathGroup.get(1).getMarkers(), eventMap)))
      .andThen(new WaitCommand(0.5)));
      // .andThen(new RunCommand(()-> intake.manipulates(-1.0), intake).withTimeout(0.5)));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
