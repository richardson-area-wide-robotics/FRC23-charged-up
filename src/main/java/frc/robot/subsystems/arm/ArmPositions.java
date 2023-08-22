package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmPositions extends SubsystemBase {
    public ArmPositions() {
        // Arm Stowed Positions
        SmartDashboard.putNumber("Arm stowed", Positions.ARM_STOWED);
        SmartDashboard.putNumber("Elbow stowed", Positions.ELBOW_STOWED);
        // Arm Intake Tipped Cone
        SmartDashboard.putNumber("Arm Tipped Cone", Positions.ARM_PICK_UP_TCONE);
        SmartDashboard.putNumber("Elbow Tipped Cone", Positions.ELBOW_PICK_UP_TCONE);
        // Arm Intake Standing Cone
        SmartDashboard.putNumber("Arm Standing Cone", Positions.ARM_PICK_UP_CONE);
        SmartDashboard.putNumber("Elbow Standing Cone", Positions.ELBOW_PICK_UP_CONE);
        // Arm Intake Cube
        SmartDashboard.putNumber("Arm Cube", Positions.ARM_PICK_UP_CUBE);
        SmartDashboard.putNumber("Elbow cube", Positions.ELBOW_PICK_UP_CUBE);
        // Score Cube Mid
        SmartDashboard.putNumber("Arm Cube Mid", Positions.ARM_SCORE_CUBE_MID);
        SmartDashboard.putNumber("Elbow Cube Mid", Positions.ELBOW_SCORE_CUBE_MID);
        // Score Cube High
        SmartDashboard.putNumber("Arm Cube High", Positions.ARM_SCORE_CUBE_HIGH);
        SmartDashboard.putNumber("Elbow Cube High", Positions.ELBOW_SCORE_CUBE_HIGH);
        // Score Cone Mid
        SmartDashboard.putNumber("Arm Cone Mid", Positions.ARM_SCORE_CONE_MID);
        SmartDashboard.putNumber("Elbow Cone Mid", Positions.ELBOW_SCORE_CONE_MID);
        // Score Cone High
        SmartDashboard.putNumber("Arm Cone High", Positions.ARM_SCORE_CONE_HIGH);
        SmartDashboard.putNumber("Elbow Cone High", Positions.ELBOW_SCORE_CONE_HIGH);
        // Elbow Idle Position
        SmartDashboard.putNumber("Elbow Idle", Positions.ELBOW_IDLE);
    }

    public static class Positions {
        // Arm Stowed Positions
        public static double ARM_STOWED = ArmConstants.ARM_STOWED;
        public static double ELBOW_STOWED = ArmConstants.ELBOW_STOWED;
        public static double ARM_PICK_UP_TCONE = ArmConstants.ARM_PICK_UP_TCONE;
        public static double ELBOW_PICK_UP_TCONE = ArmConstants.ELBOW_PICK_UP_TCONE;
        public static double ARM_PICK_UP_CONE = ArmConstants.ARM_PICK_UP_CONE;
        public static double ELBOW_PICK_UP_CONE = ArmConstants.ELBOW_PICK_UP_CONE;
        public static double ARM_PICK_UP_CUBE = ArmConstants.ARM_PICK_UP_CUBE;
        public static double ELBOW_PICK_UP_CUBE = ArmConstants.ELBOW_PICK_UP_CUBE;
        public static double ARM_SCORE_CUBE_LOW = ArmConstants.ARM_SCORE_CUBE_LOW;
        public static double ELBOW_SCORE_CUBE_LOW = ArmConstants.ELBOW_SCORE_CUBE_LOW;
        public static double ARM_SCORE_CONE_LOW = ArmConstants.ARM_SCORE_CONE_LOW;
        // .7345
        public static double ARM_SCORE_CONE_MID = ArmConstants.ARM_SCORE_CONE_MID;
        public static double ELBOW_SCORE_CONE_LOW = ArmConstants.ELBOW_SCORE_CONE_LOW;
        // .58
        public static double ELBOW_SCORE_CONE_MID = ArmConstants.ELBOW_SCORE_CONE_MID;

        public static double ARM_SCORE_CUBE_MID = ArmConstants.ARM_SCORE_CUBE_MID;
        public static double ELBOW_SCORE_CUBE_MID = ArmConstants.ELBOW_SCORE_CUBE_MID;
        // .0185
        public static double ARM_SCORE_CONE_HIGH = ArmConstants.ARM_SCORE_CONE_HIGH;
        // .32
        public static double ELBOW_SCORE_CONE_HIGH = ArmConstants.ELBOW_SCORE_CONE_HIGH;

        public static double ARM_SCORE_CUBE_HIGH = ArmConstants.ARM_SCORE_CUBE_HIGH;
        public static double ELBOW_SCORE_CUBE_HIGH = ArmConstants.ELBOW_SCORE_CUBE_HIGH;

        public static double ARM_PICK_UP_SHELF = ArmConstants.ARM_PICK_UP_SHELF;
        public static double ELBOW_PICK_UP_SHELF = ArmConstants.ELBOW_PICK_UP_SHELF;
        // Arm back cone pick up scoring position
        public static double ARM_BACK_PICKUP = 0.0;
        public static double ELBOW_BACK_PICKUP = 0.0;
        public static double ARM_SPECIAL_IDLE = 0.22;
        public static double ELBOW_SPECIAL_IDLE = 0.0;
        // Elbow Idle position
        public static double ELBOW_IDLE = ArmConstants.ELBOW_IDLE;
        
    }

    @Override
    public void periodic() {
        double armStowed = SmartDashboard.getNumber("Arm stowed", 0);
        double elbowStowed = SmartDashboard.getNumber("Elbow stowed", 0);
        double armPickUpTCone = SmartDashboard.getNumber("Arm Tipped Cone", 0);
        double elbowPickUpTCone = SmartDashboard.getNumber("Elbow Tipped Cone", 0);
        double armPickUpStandCone = SmartDashboard.getNumber("Arm Standing Cone", 0);
        double elbowPickUpStandCone = SmartDashboard.getNumber("Elbow Standing Cone", 0);
        double armPickUpCube = SmartDashboard.getNumber("Arm Cube", 0);
        double elbowPickUpCube = SmartDashboard.getNumber("Elbow cube", 0);
        double armScoreCubeMid = SmartDashboard.getNumber("Arm Cube Mid", 0);
        double elbowScoreCubeMid = SmartDashboard.getNumber("Elbow Cube Mid", 0);
        double armScoreCubeHigh = SmartDashboard.getNumber("Arm Cube High", 0);
        double elbowScoreCubeHigh = SmartDashboard.getNumber("Elbow Cube High", 0);
        double armScoreConeMid = SmartDashboard.getNumber("Arm Cone Mid", 0);
        double elbowScoreConeMid = SmartDashboard.getNumber("Elbow Cone Mid", 0);
        double armScoreConeHigh = SmartDashboard.getNumber("Arm Cone High", 0);
        double elbowScoreConeHigh = SmartDashboard.getNumber("Elbow Cone High", 0);
        double elbowIdle = SmartDashboard.getNumber("Elbow Idle", 0);

        if ((armStowed != Positions.ARM_STOWED)) {
            Positions.ARM_STOWED = armStowed;
        }
        if ((elbowStowed != Positions.ELBOW_STOWED)) {
            Positions.ELBOW_STOWED = elbowStowed;
        }
        if ((armPickUpTCone != Positions.ARM_PICK_UP_TCONE)) {
            Positions.ARM_PICK_UP_TCONE = armPickUpTCone;
        }
        if ((elbowPickUpTCone != Positions.ELBOW_PICK_UP_TCONE)) {
            Positions.ELBOW_PICK_UP_TCONE = elbowPickUpTCone;
        }
        if ((armPickUpStandCone != Positions.ARM_PICK_UP_CONE)) {
            Positions.ARM_PICK_UP_CONE = armPickUpStandCone;
        }
        if ((elbowPickUpStandCone != Positions.ELBOW_PICK_UP_CONE)) {
            Positions.ELBOW_PICK_UP_CONE = elbowPickUpStandCone;
        }
        if ((armPickUpCube != Positions.ARM_PICK_UP_CUBE)) {
            Positions.ARM_PICK_UP_CUBE = armPickUpCube;
        }
        if ((elbowPickUpCube != Positions.ELBOW_PICK_UP_CUBE)) {
            Positions.ELBOW_PICK_UP_CUBE = elbowPickUpCube;
        }
        if ((armScoreCubeMid != Positions.ARM_SCORE_CUBE_MID)) {
            Positions.ARM_SCORE_CUBE_MID = armScoreCubeMid;
        }
        if ((elbowScoreCubeMid != Positions.ELBOW_SCORE_CUBE_MID)) {
            Positions.ELBOW_SCORE_CUBE_MID = elbowScoreCubeMid;
        }
        if ((armScoreCubeHigh != Positions.ARM_SCORE_CUBE_HIGH)) {
            Positions.ARM_SCORE_CUBE_HIGH = armScoreCubeHigh;
        }
        if ((elbowScoreCubeHigh != Positions.ELBOW_SCORE_CUBE_HIGH)) {
            Positions.ELBOW_SCORE_CUBE_HIGH = elbowScoreCubeHigh;
        }
        if ((armScoreConeMid != Positions.ARM_SCORE_CONE_MID)) {
            Positions.ARM_SCORE_CONE_MID = armScoreConeMid;
        }
        if ((elbowScoreConeMid != Positions.ELBOW_SCORE_CONE_MID)) {
            Positions.ELBOW_SCORE_CONE_MID = elbowScoreConeMid;
        }
        if ((armScoreConeHigh != Positions.ARM_SCORE_CONE_HIGH)) {
            Positions.ARM_SCORE_CONE_HIGH = armScoreConeHigh;
        }
        if ((elbowScoreConeHigh != Positions.ELBOW_SCORE_CONE_HIGH)) {
            Positions.ELBOW_SCORE_CONE_HIGH = elbowScoreConeHigh;
        }
        if ((elbowIdle != Positions.ELBOW_IDLE)) {
            Positions.ELBOW_IDLE = elbowIdle;
        }

    }

}
