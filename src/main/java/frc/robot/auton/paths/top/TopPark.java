package frc.robot.auton.paths.top;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auton.util.AutonBase;
import frc.robot.auton.util.AutonUtil;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TopPark extends AutonBase {
    public TopPark(
    DriveSubsystem drive) {
      
    PathPlannerTrajectory parkingPath = AutonUtil.loadTrajectory("Top-Park", 2.0, 5.0);

    Pose2d initialPose = AutonUtil.initialPose(parkingPath);

    if (parkingPath == null) {
        System.out.println("Path not found"); //TODO: change to logging to dashboard
        return;
    }


    }
}
