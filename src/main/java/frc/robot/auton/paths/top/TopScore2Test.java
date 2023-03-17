package frc.robot.auton.paths.top;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.swerve.MAXSwerve;
import frc.robot.Constants;
import frc.robot.auton.util.AutonBase;
import frc.robot.auton.util.AutonUtil;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

public class TopScore2Test extends AutonBase {
    PositionCommand armPositions;

    public TopScore2Test(
    DriveSubsystem drive,
    Arm m_arm, Intake m_intake) {
      
    PathPlannerTrajectory PickUpOne = AutonUtil.loadTrajectory("Top-Simple-Park", 2.0, 5.0);
    this.armPositions = new PositionCommand(m_arm);

    Pose2d initialPose = AutonUtil.initialPose(PickUpOne);

    if (PickUpOne == null) {
        System.out.println("Path not found");
        return;
    }

    addCommandsWithLog("Top park",
     new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"),//, armPositions.autonArmScoreConeHighCommand(), //new RunCommand(()-> m_intake.manipulates(0.5)) , //drive.trajectoryFollowerCommand(PickUpOne),
    new InstantCommand(() -> drive.drive(0.0, 0.0, 0.0, false), drive));


    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
