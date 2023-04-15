package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

public class Localizer extends SubsystemBase {
  private AprilTagFieldLayout fieldLayout;
  private NodePositionLayout nodeLayout;
  private String filename = "/ChargedUp.json";
  private String nodePositionFilename = "/ScoringLocations.json";
  private PhotonCamera camera;
  private Optional<Transform3d> currentAprilTagTransform = Optional.empty();
  private Optional<Integer> currentAprilTagID = Optional.empty();
  private Optional<Double> currentTimeStamp = Optional.empty();
  private Transform3d cameraPos;

  public Localizer(String name, Transform3d cameraPositon) throws IOException {
    String path = Filesystem.getDeployDirectory().getPath() + filename;
    fieldLayout = new AprilTagFieldLayout(path);

    String nodePositionPath = Filesystem.getDeployDirectory().getPath() + nodePositionFilename;
    nodeLayout = new NodePositionLayout(nodePositionPath);
    var alliance = DriverStation.getAlliance();
    // fieldLayout.setOrigin(alliance == Alliance.Blue ?
    //     OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);

    camera = new PhotonCamera(name);
    cameraPos = cameraPositon;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (target.getPoseAmbiguity() <= 0.2){
      currentAprilTagID = Optional.of(target.getFiducialId());
      currentAprilTagTransform = Optional.of(target.getBestCameraToTarget());
      currentTimeStamp = Optional.of(result.getTimestampSeconds());
      SmartDashboard.putString("tag", "" + target.getFiducialId());
      SmartDashboard.putNumber("PoseX", getRobotPose().get().getX());
      SmartDashboard.putNumber("PoseY", getRobotPose().get().getY());
    }
    } else {
      currentAprilTagTransform = Optional.empty();
      currentAprilTagID = Optional.empty();
      currentTimeStamp = Optional.empty();
    }

  }

  public void start() {
  }

  public Optional<Pose3d> getRobotPose() {
    Optional<Pose3d> posePlus = Optional.empty();
    if (currentAprilTagTransform.isPresent() && currentAprilTagID.isPresent()) {
      posePlus = Optional.of(findPoseTransform(currentAprilTagTransform.get(), currentAprilTagID.get()));
    }

    return posePlus;
  }

  private Pose3d findPoseTransform(Transform3d cameraToTarget, int tagID) {
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

    Pose3d robotPosition = new Pose3d();
    if (tagPose.isPresent()) {
      Pose3d camPose = tagPose.get().transformBy(cameraToTarget.inverse());
      // robotPosition = tagPose.get().transformBy(cameraToTarget);
      SmartDashboard.putNumber("camPoseX", camPose.getX());
      SmartDashboard.putNumber("camPoseY", camPose.getY());
      robotPosition = camPose.transformBy(cameraPos);

    }
    return robotPosition;
  }

  public Optional<Double> getPoseTimeStamp() {
    return currentTimeStamp;
  }

  // TODO future work Add method to find pose/transform for desired position in
  // front of node
}

