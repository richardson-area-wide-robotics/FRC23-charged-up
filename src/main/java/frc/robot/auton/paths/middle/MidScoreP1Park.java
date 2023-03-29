package frc.robot.auton.paths.middle;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
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

public class MidScoreP1Park extends AutonBase {
    public PositionCommand armPositions;
    public BalancingCommand balance;
    
    public MidScoreP1Park(
    DriveSubsystem drive, 
    Intake intake,
    Arm m_arm){

    List<PathPlannerTrajectory> midP1Park = AutonUtil.loadTrajectoryGroup("Mid-Score-P1Park", new PathConstraints(2.5, 3.5));
    PathPlannerTrajectory firstPath = midP1Park.get(0);

    Pose2d initialPose = AutonUtil.initialPose(firstPath);
    this.armPositions = new PositionCommand(m_arm);
    this.balance = new BalancingCommand(drive);

     /*
     * Checks if the paths are null and if they are it will print out that the path was not found
     */
    if (firstPath == null) {
        System.out.println("Path not found");
        return;
    }

    /**
     * Creates a command group that this auton class will call on when initialized 
     */
    addCommandsWithLog("Mid P1 Park",
    /* Runs commands to score pre-load Cone in high */
      new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"),
        new RunCommand(()-> intake.manipulates(-1.0), intake)
          .raceWith(armPositions.autonArmScoreConeHighCommand())
            .andThen(new WaitCommand(0.1))
              .andThen(new RunCommand(()-> intake.manipulates(0.25), intake).withTimeout(0.5))

      /*
       * Stows the arm 
       * Waits for 0.5 seconds
       * Follows the first path
       */
      .andThen(armPositions.armStowCommand())
        .andThen(new RunCommand(()-> intake.manipulates(1.0), intake)
          .raceWith(AutonUtil.followEventCommand(drive.trajectoryFollowerCommand(firstPath), firstPath)))
      
      /*
       * Activates the balancing command
       */
      .andThen(balance));
    }

    @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

 }
