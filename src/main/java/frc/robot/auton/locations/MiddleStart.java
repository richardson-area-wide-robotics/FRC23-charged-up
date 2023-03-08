package frc.robot.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MiddleStart extends Locations {
    private MiddleStart() {}

    @Override
    public Pose2d get() {
        return new Pose2d(1.81, 2.21, Rotation2d.fromDegrees(180));
    }

    public static final MiddleStart instance = new MiddleStart();
    
}
