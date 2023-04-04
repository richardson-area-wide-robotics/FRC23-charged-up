package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        if (!arm.getNormalStow()){
        return new InstantCommand(()->arm.setElbowPosition(ArmPositions.Positions.ELBOW_STOWED)).alongWith(new ShoulderPosition(arm, ArmPositions.Positions.ARM_SPECIAL_IDLE)).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED));
        // return new InstantCommand(()->arm.setElbowPosition(ArmPositions.Positions.ELBOW_STOWED)).andThen(new WaitUntilCommand(null))
        }
        else if (arm.getNormalStow()){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED)).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_STOWED)));
        }
        else {
            return new InstantCommand(()->arm.setElbowPosition(ArmPositions.Positions.ELBOW_STOWED)).alongWith(new ShoulderPosition(arm, ArmPositions.Positions.ARM_SPECIAL_IDLE)).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED));
        }
    }

    public Command armPickUpTConeComand(){
        return new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_TCONE)).until(()->isFinished()).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_TCONE)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armPickUpConeCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CONE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_CONE)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armPickUpCubeCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CUBE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_CUBE)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command autonArmPickUpCubeCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CUBE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_CUBE - .025)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armPickUpFromShelf(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_SHELF).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_SHELF)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armPickUpFromDoubleShelf(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_DOUBLE_SHELF).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_DOUBLE_SHELF)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armScoreCubeMidCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_MID)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armScoreCubeHighCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_HIGH)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armScoreConeMidCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_MID)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command autonArmScoreConeMidCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_MID - 0.01)).alongWith(new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command armScoreConeHighCommand(){
        // return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(arm.getArmPosition()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH));
        return new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_SPECIAL_IDLE)).until(()-> isFinished()).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).alongWith(new InstantCommand(()-> {arm.setElbowPosition(ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH);
        arm.setNormalStow(false);})));
    }

    public Command armBackStandingCone(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SPECIAL_IDLE), new InstantCommand(()-> arm.setNormalStow(true)));
    }

    public Command autonArmScoreConeHighCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE + 0.06).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH));
    }

}
