// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Camera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
  static PhotonCamera camera;

  public Camera() {
    Camera.camera = new PhotonCamera("slotheye");
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Transform3d getDistanceAndAngle() {
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d pose = target.getBestCameraToTarget();
    
    return pose;
    
  }

  public static double getAngle() {
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double  angle = target.getYaw();

    return angle;
  }
}