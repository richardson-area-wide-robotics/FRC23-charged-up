package frc.robot.auton.paths.middle;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

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

public class MidScoreP1Park extends AutonBase {
    public PositionCommand armPositions;
    public BalancingCommand balance;
    
    public MidScoreP1Park(
    DriveSubsystem drive, 
    Intake intake,
    Arm m_arm){

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Mid-Score-P1-Park", new PathConstraints(2.5, 3.5));
    HashMap<String, Command> eventMap = new HashMap<>();

    Pose2d initialPose = AutonUtil.initialPose(pathGroup.get(0));
    this.armPositions = new PositionCommand(m_arm);
    this.balance = new BalancingCommand(drive);

    eventMap.put("Stow", armPositions.armStowCommand());
    eventMap.put("IntakeDown", armPositions.armPickUpCubeCommand());

    if (pathGroup.get(0) == null) {
        System.out.println("Path not found");
        return;
    }

    addCommandsWithLog("Mid Score+1 and Park",
      new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"),
      new RunCommand(()-> intake.manipulates(-1.0), intake)
      .raceWith(armPositions.autonArmScoreConeHighCommand())
      .andThen(new WaitCommand(0.1))
      .andThen(new RunCommand(()-> intake.manipulates(0.25), intake).withTimeout(0.5))
      .andThen(armPositions.armStowCommand())
      .andThen(new WaitCommand(0.5))
      .andThen(new FollowPathWithEvents(drive.trajectoryFollowerCommand(pathGroup.get(0)), pathGroup.get(0).getMarkers(), eventMap))
      .andThen(balance)
      .andThen(new InstantCommand(() -> drive.drive(0.0, 0.0, 0.0, false), drive)));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

 }