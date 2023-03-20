package frc.robot.auton.paths.middle;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
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

public class MiddlePark extends AutonBase {
    public PositionCommand armPositions;
    public BalancingCommand balance;
    
    public MiddlePark(DriveSubsystem drive, Intake intake,
    Arm m_arm){

        //PathPlannerTrajectory PickUpOne = AutonUtil.loadTrajectory("Top-Simple-Park", 2.0, 5.0);
    //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Top-Score-Three", new PathConstraints(2.0, 5.0));
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Top-Score", new PathConstraints(1.5, 3.5));

    Pose2d initialPose = AutonUtil.initialPose(pathGroup.get(0));
    this.armPositions = new PositionCommand(m_arm);
    this.balance = new BalancingCommand(drive);

    if (pathGroup.get(0) == null) {
        System.out.println("Path not found");
        return;
    }

    addCommandsWithLog("Top park",
      new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"),
      new RunCommand(()-> intake.manipulates(-1.0), intake)
      .raceWith(armPositions.autonArmScoreConeHighCommand())
      .andThen(new WaitCommand(0.1))
      .andThen(new RunCommand(()-> intake.manipulates(0.25), intake).withTimeout(0.5))
      .andThen(armPositions.armStowCommand())
      .andThen(new WaitCommand(0.5))
      .andThen(drive.trajectoryFollowerCommand(pathGroup.get(0)))
      .andThen(balance).andThen(new InstantCommand(() -> drive.drive(0.0, 0.0, 0.0, false), drive)));
    }

  // public SequentialCommandGroup scoringFirst(){
  //   return new SequentialCommandGroup(armPositions.autonArmScoreConeHighCommand().alongWith(new RunCommand(()-> intake.manipulates(-0.5))), new WaitCommand(1.7), new RunCommand(()-> intake.manipulates(1)), new WaitCommand(0.5), new InstantCommand(()->intake.stop())); 
  // }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

 }
