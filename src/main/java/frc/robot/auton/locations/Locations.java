package frc.robot.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class Locations {
    public abstract Pose2d get();

  public String getName() {
    String name = this.getClass().getSimpleName();
    return name.substring(name.lastIndexOf('.') + 1);
  }

  public static TopStart TopStart(){
    return TopStart.instance;
  }

}
