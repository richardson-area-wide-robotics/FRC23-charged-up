package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Localizer {
  private Thread visionThread;
  private AprilTagDetector detector;

  public Localizer() {
    detector = new AprilTagDetector();
    detector.addFamily("tag16h5", 0);
    AprilTagDetector.Config config = new AprilTagDetector.Config();
    detector.setConfig(config);

    visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();

              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();

              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();
              Mat grayMat = new Mat();
              ArrayList<Integer> tags = new ArrayList<>();
              AprilTagPoseEstimator.Config poseEstConfig =
                  new AprilTagPoseEstimator.Config(
                      Constants.Localizer.TARGET_SIZE_METERS,
                      Constants.Localizer.FX_PIXELS,
                      Constants.Localizer.FY_PIXELS,
                      Constants.Localizer.CX_PIXELS,
                      Constants.Localizer.CY_PIXELS);
              AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {

                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }

                Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);
                AprilTagDetection[] detections = detector.detect(grayMat);
                tags.clear();
                for (AprilTagDetection detection : detections) {
                  draw(mat, detection);
                  Transform3d pose = estimator.estimate(detection);
                  SmartDashboard.putString("tag", "" + detection.getId());
                  SmartDashboard.putString("pose", pose.toString());
                }

                // Put a rectangle on the image
                // Imgproc.rectangle(
                //     mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
  }

  public void start() {
    visionThread.setDaemon(true);
    visionThread.start();
  }

  private void draw(Mat mat, AprilTagDetection tag) {
    //double[] corners = tag.getCorners();
    //Point prev = new Point(corners[6], corners[7]);
    for (int ind = 0; ind <= 3; ind++) {
      int end = (ind + 1) % 4;
      Point point1 = new Point(tag.getCornerX(ind), tag.getCornerY(ind));
      Point point2 = new Point(tag.getCornerX(end), tag.getCornerY(end));
      Imgproc.line(mat, point1, point2, new Scalar(255, 0, 0), 2);
    }
  }
}
