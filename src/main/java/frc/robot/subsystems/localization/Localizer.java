package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Localizer extends SubsystemBase{
  private Thread visionThread;
  private AprilTagDetector detector;
  private AprilTagFieldLayout fieldLayout;
  private NodePositionLayout nodeLayout;
  private String filename = "/ChargedUp.json";
  private String nodePositionFilename = "/ScoringLocations.json";
  private AprilTagDetection[] detections;
  private AprilTagPoseEstimator estimator;
  int count;

  public Localizer() throws IOException {
    detector = new AprilTagDetector();
    detector.addFamily("tag16h5", 0);
    AprilTagDetector.Config config = new AprilTagDetector.Config();
    detector.setConfig(config);
    String path = Filesystem.getDeployDirectory().getPath() + filename;
    fieldLayout = new AprilTagFieldLayout(path);

    String nodePositionPath = Filesystem.getDeployDirectory().getPath() + nodePositionFilename;
    nodeLayout = new NodePositionLayout(nodePositionPath);
    
PhotonCamera camera = new PhotonCamera("Slotheye");

    visionThread =
        new Thread(
            () -> {

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {

                PhotonPipelineResult result = camera.getLatestResult();
                PhotonTrackedTarget target = result.getBestTarget();

                if(target!=null)
                {

                  SmartDashboard.putNumber("tag seen", target.getFiducialId());

                  int tagID = target.getFiducialId();
                  Transform3d transformPose = target.getBestCameraToTarget();
                  Pose3d plusPose = findPoseTransform(transformPose, tagID);
                  SmartDashboard.putString("tag", "" + tagID);         
              }
              }
            });

  }

  public void start() {
    visionThread.setDaemon(true);
    visionThread.start();
  }

  private Pose3d findPoseTransform(Transform3d pose, int tagID){ 
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

    Pose3d robotPosition = new Pose3d();
    if(tagPose.isPresent()){
      robotPosition = tagPose.get().transformBy(pose);
  }
    // Returns an empty pose3d if not tag is seen
    return robotPosition;
  }

  public Transform3d getAprilTagBasedTransform(int aprilTagId,  int gridId)
  {
    Transform3d transform = new Transform3d();

    for (AprilTagDetection detection : detections) {
      if(detection.getId() == aprilTagId)
      {
        Transform3d pose = estimator.estimate(detection);
        Pose3d transformPose = findPoseTransform(pose, aprilTagId);

        Optional<Pose3d> nodePose = nodeLayout.getPositionPose(gridId);

        if(nodePose.isPresent())
        {
          transform = transformPose.minus(nodePose.get());
        }
      }
    }


    return transform;
  }
}
