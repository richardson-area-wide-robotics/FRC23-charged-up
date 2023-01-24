// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Camera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
  PhotonCamera camera;

  public Camera() {
    this.camera = new PhotonCamera("slotheye");
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
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getAngle() {
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d pose = target.getBestCameraToTarget();
    Rotation3d rotation = pose.getRotation();
    double angle = rotation.getAngle();
    double xRot = rotation.getX();
    System.out.println("Angle: " + angle*180/3.14);
    System.out.println("XRot: " + xRot*180/3.14);
    return angle;
  }
}