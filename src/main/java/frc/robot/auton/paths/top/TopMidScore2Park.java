package frc.robot.auton.paths.top;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.util.AutonBase;
import frc.robot.auton.util.AutonUtil;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

public class TopMidScore2Park extends AutonBase {
  public PositionCommand armPositions;

    public TopMidScore2Park(
    DriveSubsystem drive,
    Arm m_arm, Intake intake) {
      
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Top-Score-2-Mid", new PathConstraints(3.0, 5.0), new PathConstraints(2.0, 3.0));

    Pose2d initialPose = AutonUtil.initialPose(pathGroup.get(0));
    this.armPositions = new PositionCommand(m_arm);
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", armPositions.armPickUpCubeCommand());
    eventMap.put("intake", new RunCommand(()->intake.manipulates(0.75), intake));

    if (pathGroup.get(0) == null && pathGroup.get(1) == null) {
        System.out.println("Path not found");
        return;
    }

    addCommandsWithLog("Top park",
      new RunCommand(()-> intake.manipulates(-1.0), intake)
      .raceWith(armPositions.armScoreConeMidCommand())
      .andThen(new WaitCommand(0.5))
      .andThen(new RunCommand(()-> intake.manipulates(0.25), intake).withTimeout(0.5))
      .andThen(armPositions.armStowCommand())
      .andThen(armPositions.armPickUpCubeCommand())
      .andThen(new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"))
      .andThen(new RunCommand(()-> intake.manipulates(1.0), intake)
      .raceWith(drive.trajectoryFollowerCommand(pathGroup.get(0))))
      .andThen(armPositions.armStowCommand())
      .andThen(new InstantCommand(()-> intake.manipulates(0)))
      .andThen(drive.trajectoryFollowerCommand(pathGroup.get(1)))
      .andThen(armPositions.armScoreCubeMidCommand())
      .andThen(new WaitCommand(0.5))
      .andThen(new RunCommand(()-> intake.manipulates(-1.0), intake).withTimeout(1.0))
      .andThen(armPositions.armStowCommand())
      .andThen(drive.trajectoryFollowerCommand(pathGroup.get(2))));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}