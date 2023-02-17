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

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Localizer {
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

    AprilTagPoseEstimator.Config poseEstConfig =
    new AprilTagPoseEstimator.Config(
        Constants.TARGET_SIZE_METERS,
        Constants.FX_PIXELS,
        Constants.FY_PIXELS,
        Constants.CX_PIXELS,
        Constants.CY_PIXELS);
estimator = new AprilTagPoseEstimator(poseEstConfig);


PhotonCamera camera = new PhotonCamera("Slotheye");



    visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer

              SmartDashboard.putBoolean("Camera ON", true);
              SmartDashboard.putNumber("camera Count", count++);
              // Set the resolution

              // // Get a CvSink. This will capture Mats from the camera
              // CvSink cvSink = CameraServer.getVideo();

              // // Setup a CvSource. This will send images back to the Dashboard
              // CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // // Mats are very memory expensive. Lets reuse this Mat.
              // Mat mat = new Mat();
              // Mat grayMat = new Mat();
              // ArrayList<Integer> tags = new ArrayList<>();


              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {

                // // Tell the CvSink to grab a frame from the camera and put it
                // // in the source mat.  If there is an error notify the output.
                // if (cvSink.grabFrame(mat) == 0) {
                //   // Send the output the error.
                //   outputStream.notifyError(cvSink.getError());
                //   // skip the rest of the current iteration
                //   continue;
                // }

                // Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);
                // detections = detector.detect(grayMat);

                PhotonPipelineResult result = camera.getLatestResult();
                PhotonTrackedTarget target = result.getBestTarget();

                if(target!=null)
                {

                  SmartDashboard.putNumber("tag seen", target.getFiducialId());

                  int tagID = target.getFiducialId();
                  Transform3d transformPose = target.getBestCameraToTarget();
                  Pose3d plusPose = findPoseTransform(transformPose, tagID);
                  SmartDashboard.putString("tag", "" + tagID);
                  SmartDashboard.putString("transform", transformPose.toString());
                  SmartDashboard.putString("plus", plusPose.toString());                
              }

                // Put a rectangle on the image
                // Imgproc.rectangle(
                //     mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

                // Give the output stream a new image to display
                // outputStream.putFrame(mat);
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
      
    SmartDashboard.putString("tag pose", tagPose.toString());
    
      robotPosition = tagPose.get().transformBy(pose);
  }
    return robotPosition;
  }

  private Pose3d findPosePlus(Transform3d pose, int tagID){ 
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

    SmartDashboard.putString("tag pose", tagPose.toString());

    Pose3d robotPosition = new Pose3d();
    if(tagPose.isPresent()){
      robotPosition = tagPose.get().plus(pose);
  }
    return robotPosition;
  }

  //TODO: Figure out if minus works, See if robot goes to correct team community, double check the .47 measurement, test if findPosePlus or findPoseTransform are correct. 

  public Transform3d getAprilTagBasedTransform(int aprilTagId,  int gridId)
  {
    Transform3d transform = new Transform3d();

    for (AprilTagDetection detection : detections) {
      if(detection.getId() == aprilTagId)
      {
        //draw(mat, detection);
        Transform3d pose = estimator.estimate(detection);
        Pose3d transformPose = findPoseTransform(pose, aprilTagId);
        // Pose3d plusPose = findPosePlus(pose, tagID);

        Optional<Pose3d> nodePose = nodeLayout.getPositionPose(gridId);

        if(nodePose.isPresent())
        {
          transform = transformPose.minus(nodePose.get());
        }
      }
    }


    return transform;
  }


  //private void draw(Mat mat, AprilTagDetection tag) {
    //for (int ind = 0; ind < 4; ind++) {
      //int end = ind % 4;
      //Point pointA = new Point(getCornerX(ind), getCornerY(ind + 1));
      //Point pointB = new Point(getCornerX(end), getCornerY(end + 1));
      //Imgproc.line(mat, pointA, pointB, new Scalar(255, 0, 0), 3);
    //}
  //}
}
