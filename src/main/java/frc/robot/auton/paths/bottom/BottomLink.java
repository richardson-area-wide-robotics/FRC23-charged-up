package frc.robot.auton.paths.bottom;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
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

public class BottomLink extends AutonBase {
    public PositionCommand armPositions;
    public BalancingCommand balance;
    
    public BottomLink(
    DriveSubsystem drive, 
    Intake intake,
    Arm m_arm){

      List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Bottom-Score-3-Test", new PathConstraints(1.5, 3.5), new PathConstraints(2.0, 4.0));
  
      Pose2d initialPose = AutonUtil.initialPose(pathGroup.get(0));
      this.armPositions = new PositionCommand(m_arm);
      this.balance = new BalancingCommand(drive);
  
      if (pathGroup.get(0) == null) {
          System.out.println("Path not found");
          return;
      }
  
      addCommandsWithLog("Bottom Score 3",
        new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"),
        new RunCommand(()-> intake.setIntakeSpeed(-1.0), intake)
        .raceWith(armPositions.armScoreConeMidCommand())
        .andThen(new WaitCommand(0.1))
        .andThen(new RunCommand(()-> intake.setIntakeSpeed(0.25), intake).withTimeout(0.5))
        .andThen(new WaitCommand(0.5))
        .andThen(new RunCommand(()-> intake.setIntakeSpeed(-1.0), intake))
        .raceWith(new FollowPathWithEvents(drive.trajectoryFollowerCommand(pathGroup.get(0)), pathGroup.get(0).getMarkers(), AutonUtil.getEventMap()))
        .andThen(new WaitCommand(0.2))
        .andThen(new RunCommand(()-> intake.setIntakeSpeed(1.0), intake)).withTimeout(0.3)
        .andThen(new RunCommand(()-> intake.setIntakeSpeed(1.0), intake))
        .raceWith(new FollowPathWithEvents(drive.trajectoryFollowerCommand(pathGroup.get(1)), pathGroup.get(1).getMarkers(), AutonUtil.getEventMap()))
        .andThen(new WaitCommand(0.4))
        .andThen(new RunCommand(()-> intake.setIntakeSpeed(-1.0), intake)));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

 }

