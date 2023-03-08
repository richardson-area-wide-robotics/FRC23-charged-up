package frc.robot.auton.paths.top;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;

import frc.robot.subsystems.drive.DriveSubsystem;

public class TopScore {
    DriveSubsystem drive;
    public TopScore(){
// This will load the file "Example Path Group.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Example Path Group", new PathConstraints(4, 3));

// This will load the file "Example Path Group.path" and generate it with different path constraints for each segment
List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup(
    "Example Path Group", 
    new PathConstraints(4, 3), 
    new PathConstraints(2, 2), 
    new PathConstraints(3, 3));
    }
}
