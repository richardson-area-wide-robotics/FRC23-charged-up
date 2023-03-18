package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

public class Localizer extends SubsystemBase{
  private AprilTagFieldLayout fieldLayout;
  private String filename = "/ChargedUp.json";
  private PhotonCamera camera;
  private Optional<Transform3d> currentAprilTagTransform;
  private Optional<Integer> currentAprilTagID;

  public Localizer() throws IOException {
    String path = Filesystem.getDeployDirectory().getPath() + filename;
    fieldLayout = new AprilTagFieldLayout(path);
    
    camera = new PhotonCamera("Slotheye");
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();

    if(target!=null)
    {

      SmartDashboard.putNumber("tag seen", target.getFiducialId());

      currentAprilTagID = Optional.of(target.getFiducialId());
      currentAprilTagTransform = Optional.of(target.getBestCameraToTarget());
      SmartDashboard.putString("tag", "" + target.getFiducialId());         
  }
  else{
    currentAprilTagID = Optional.of(null);
    currentAprilTagTransform = Optional.of(null);
  }
  }

  public void start() {
  }

  public Pose3d getRobotPose()
  {
    Pose3d posePlus = new Pose3d();
    if(currentAprilTagTransform.isPresent() && currentAprilTagID.isPresent())
    {
      posePlus = findPoseTransform(currentAprilTagTransform.get(), currentAprilTagID.get());
    }

    return posePlus;
  }

  private Pose3d findPoseTransform(Transform3d pose, int tagID){ 
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

    Pose3d robotPosition = new Pose3d();
    if(tagPose.isPresent()){
      robotPosition = tagPose.get().transformBy(pose);
  }
    return robotPosition;
  }

  //TODO future work Add method to find pose/transform for desired position in front of node
}
