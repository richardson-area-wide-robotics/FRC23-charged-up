package frc.robot.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BottomStart extends Locations{
    private BottomStart(){}

    @Override
    public Pose2d get() {
        return new Pose2d(1.81, 0.44, Rotation2d.fromDegrees(180));
    }

    public static final BottomStart instance = new BottomStart();
    
}
