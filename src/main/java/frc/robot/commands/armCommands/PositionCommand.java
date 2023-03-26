package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.intake.Intake;

public class PositionCommand extends SequentialCommandGroup {
    ElbowPosition elbowCommand;
    ShoulderPosition shoulderPosition;
    Arm arm;
    Intake intake;

    public PositionCommand(Arm armMech){
        this.arm = armMech;
    }

    public Command armStowCommand(){
        if (arm.getArmAbsoluteEncoder() > 0.25){
        return new InstantCommand(()->arm.setElbowPosition(ArmPositions.Positions.ELBOW_STOWED)).alongWith(new ShoulderPosition(arm, ArmPositions.Positions.ARM_SPECIAL_IDLE)).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED));
        // return new InstantCommand(()->arm.setElbowPosition(ArmPositions.Positions.ELBOW_STOWED)).andThen(new WaitUntilCommand(null))
        }
        else{
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED)).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_STOWED)));
        }
    }

    public Command armPickUpTConeComand(){
        return new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_TCONE)).until(()->isFinished()).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_TCONE));
    }

    public Command armPickUpConeCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CONE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_CONE));
    }

    public Command armPickUpCubeCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CUBE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_CUBE));
    }

    public Command armPickUpFromShelf(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_SHELF).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_SHELF));
    }

    public Command armScoreCubeMidCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_MID));
    }

    public Command armScoreCubeHighCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_HIGH));
    }

    public Command armScoreConeMidCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_MID));
    }

    public Command armScoreConeHighCommand(){
        // return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(arm.getArmPosition()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH));
        return new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_SPECIAL_IDLE)).until(()-> isFinished()).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).alongWith(new InstantCommand(()->arm.setElbowPosition(ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH))));
    }

    public Command armBackStandingCone(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SPECIAL_IDLE));
    }

    public Command autonArmScoreConeHighCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE + 0.06).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH));
    }

}
