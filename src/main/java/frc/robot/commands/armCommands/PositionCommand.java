package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.intake.Intake;

public class PositionCommand extends SequentialCommandGroup {
    ElbowPosition elbowCommand;
    ShoulderPosition shoulderPosition;
    Arm arm;
    Intake intake;

    public PositionCommand(Arm armMech, Intake intakeMech){
        this.arm = armMech;
        this.intake = intakeMech;
    }
    
    // public SequentialCommandGroup armStowCommand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_STOWED));
    // }

    // public SequentialCommandGroup armPickUpTConeComand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_TCONE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_TCONE));
    // }

    // public SequentialCommandGroup armPickUpConeCommand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CONE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_CONE));
    // }

    // public SequentialCommandGroup armPickUpCubeCommand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_CUBE).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_CUBE));
    // }

    // public SequentialCommandGroup armPickUpFromShelf(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_SHELF).until(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_SHELF));
    // }

    // public SequentialCommandGroup armScoreCubeMidCommand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_MID));
    // }

    // public SequentialCommandGroup armScoreCubeHighCommand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CUBE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CUBE_HIGH));
    // }

    // public SequentialCommandGroup armScoreConeMidCommand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_MID).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_MID));
    // }

    // public SequentialCommandGroup armScoreConeHighCommand(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH));
    // }

    // public SequentialCommandGroup armBackStandingCone(){
    //     return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SPECIAL_IDLE));
    // }

    public Command armStowCommand(){
        if (arm.getLastArmPosition() == ArmPositions.Positions.ARM_BACK_PICKUP){
        return new ShoulderPosition(arm, ArmPositions.Positions.ARM_SPECIAL_IDLE).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SPECIAL_IDLE)).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED)).until(()->isFinished()).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_STOWED));
        }
        else{
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_STOWED)).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_STOWED)));
        }
    }

    public Command armPickUpTConeComand(){
        if (arm.getLastArmPosition() == ArmPositions.Positions.ARM_STOWED){
        return new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_TCONE)).until(()->this.arm.getArmAbsoluteEncoder() == .4).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_TCONE));
        } else if (arm.getLastArmPosition() == ArmPositions.Positions.ARM_BACK_PICKUP){
        return new ShoulderPosition(arm, ArmPositions.Positions.ARM_SPECIAL_IDLE).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SPECIAL_IDLE)).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_TCONE)).until(()->this.arm.getArmAbsoluteEncoder() == .4).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_TCONE));
        } else {
        return new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).andThen(new ShoulderPosition(arm, ArmPositions.Positions.ARM_PICK_UP_TCONE)).until(()->isFinished()).andThen(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_PICK_UP_TCONE));
        }
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
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_IDLE).until(()-> isFinished()), new ShoulderPosition(arm, ArmPositions.Positions.ARM_SCORE_CONE_HIGH).unless(()-> isFinished()), new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SCORE_CONE_HIGH));
    }

    public Command armBackStandingCone(){
        return new SequentialCommandGroup(new ElbowPosition(arm, ArmPositions.Positions.ELBOW_SPECIAL_IDLE));
    }

}
