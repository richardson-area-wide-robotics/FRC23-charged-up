package frc.robot.auton.paths.top;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auton.util.AutonBase;
import frc.robot.auton.util.AutonUtil;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

public class TopComplexAuto extends AutonBase {
    Pose2d startingPose;
    boolean shoudRobotPark;
    public TopComplexAuto(
        DriveSubsystem drive,
        Arm arm, 
        Intake intake,
        int gamePieces){

        super(gamePieces + "Top Auto");
        assert gamePieces == 0 || gamePieces == 1 || gamePieces == 2 || gamePieces == 3 || gamePieces == 4 || gamePieces == 5;


        /* Create the paths for the robot  */
        PathPlannerTrajectory parkingPath = AutonUtil.loadTrajectory("Top-Park", 2.0, 5.0);
        PathPlannerTrajectory firstPath = AutonUtil.loadTrajectory("Top-Pick-One", 2.0, 5.0);
        PathPlannerTrajectory secondPath = AutonUtil.loadTrajectory("Top-Pick-Two", 2.0, 5.0);
        PathPlannerTrajectory thirdPath = AutonUtil.loadTrajectory("Top-Pick-Three", 2.0, 5.0);
        PathPlannerTrajectory fourthPath = AutonUtil.loadTrajectory("Top-Pick-Fourth", 2.0, 5.0);
        /* Transform the paths if alliance number changed */

        if (gamePieces == 0) {
        startingPose = AutonUtil.initialPose(parkingPath); 
        } else {
        startingPose = AutonUtil.initialPose(firstPath);
        }

        if (parkingPath == null || firstPath == null || secondPath == null || thirdPath == null || fourthPath == null) {
            System.out.println("Some or all paths are not found");
            return;
          }

        /* This command is used with all autonomous routines regardless of gamepiece amount */
        addCommandsWithLog("Score-Preload", 
        new InstantCommand(() -> drive.resetOdometry(startingPose), drive).withName("Reset Odometry")/*, arm code to move the arm to high scoring position goes here, then code to outtake cone goes here, then code to stow arm goes here */);

        if (gamePieces == 0 && shoudRobotPark) {
        addCommandsWithLog("Just-Park", 
        drive.trajectoryFollowerCommand(parkingPath)
              .andThen(() -> drive.stop()) /*, code for robot driving and staying balanced on the charging path */
        );
        }

        if (gamePieces >= 1) {
        addCommandsWithLog("Pick-Up-One", drive.trajectoryFollowerCommand(firstPath));
        }

        if (gamePieces >= 2) {
            addCommandsWithLog("Pick-Up-One", null);
        }

        if (gamePieces >= 3) {
            addCommandsWithLog("Pick-Up-One", null);
        }

        if (gamePieces >= 4) {
            addCommandsWithLog("Pick-Up-One", null);
        }

        if (gamePieces >= 5) {
            addCommandsWithLog("Pick-Up-One", null);
        }

        /* Last command for parking, don't know how this will be do which is why TODO: will be to figure out how to implement this */

         }
    
}