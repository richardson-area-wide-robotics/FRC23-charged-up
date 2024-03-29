// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {

  PhotonCamera camera;

  public Camera(String name) {
    this.camera = new PhotonCamera(name);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //does this work?
  public PhotonPipelineResult getCameraResult() {
     PhotonPipelineResult result = camera.getLatestResult();
    return result;
  }

 
  public PhotonTrackedTarget getCameraTarget() {
    PhotonPipelineResult result = this.getCameraResult();
    PhotonTrackedTarget target = result.getBestTarget();    
    return target;
  }

  public Transform3d getPose() {
    PhotonTrackedTarget target = this.getCameraTarget();
    Transform3d pose = target.getBestCameraToTarget();
    return pose;
  }

  public Rotation3d getCameraRotation() {
    Transform3d pose = this.getPose();
    Rotation3d rotation = pose.getRotation();
    return rotation;
  }
/**
 * Returns the angle of the robot relative to a target in radians.
 * Convert to degrees as necessary.
 */
  public double getAngle() {
    PhotonTrackedTarget target = getCameraTarget(); 

    if(target == null) {
        return 0.0;
    }
    else {  
    double angle = Math.toRadians(target.getYaw());
    return angle;
    }
  }
}