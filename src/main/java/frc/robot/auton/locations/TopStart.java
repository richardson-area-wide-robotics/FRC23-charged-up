package frc.robot.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TopStart extends Locations{
    private TopStart(){}

    @Override
    public Pose2d get() {
        return new Pose2d(1.81, 4.94, Rotation2d.fromDegrees(180));
    }

    public static final TopStart instance = new TopStart();
    
}
