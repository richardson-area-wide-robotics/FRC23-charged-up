package frc.robot.subsystems.Camera;

import java.util.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;

public class GetDistanceAndAngle {
    XboxController xboxController = new XboxController(0);

    PhotonCamera camera = new PhotonCamera("photonvision");
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d pose = target.getBestCameraToTarget();
    double x = pose.getX();
    double y = pose.getY();
    double z = pose.getZ();
    List<TargetCorner> corners = target.getDetectedCorners();

    public boolean hasImage() {
        if (this.result != null) {
            System.out.println("Got image");
            return true;
        } else {
            return false;
        }
    }
}