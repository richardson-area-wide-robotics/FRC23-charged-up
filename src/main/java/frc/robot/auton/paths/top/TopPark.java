package frc.robot.auton.paths.top;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swerve.MAXSwerve;
import frc.robot.Constants;
import frc.robot.auton.util.AutonBase;
import frc.robot.auton.util.AutonUtil;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TopPark extends AutonBase {
    public TopPark(
    DriveSubsystem drive) {
      
    PathPlannerTrajectory parkingPath = AutonUtil.loadTrajectory("Top-Simple-Park", 2.0, 5.0);

    Pose2d initialPose = AutonUtil.initialPose(parkingPath);

    if (parkingPath == null) {
        System.out.println("Path not found");
        return;
    }

    addCommandsWithLog("Top park",
     new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"), drive.trajectoryFollowerCommand(
        parkingPath,
        Constants.AutoConstants.kPXController,
        Constants.AutoConstants.kPYController,
        Constants.AutoConstants.kPThetaController),
    new InstantCommand(() -> drive.drive(0.0, 0.0, 0.0, false), drive));


    }
    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
