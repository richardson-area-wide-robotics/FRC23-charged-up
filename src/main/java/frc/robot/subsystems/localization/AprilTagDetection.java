package frc.robot.subsystems.localization;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.apriltag.*;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class AprilTagDetection {
    private Thread visionThread;
    private AprilTagDetector detector;


    public AprilTagDetection() {
        detector = new AprilTagDetector();

        visionThread = new Thread(() -> {
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

            AprilTagDetection[] tags = detector.detect(mat);
            for (AprilTagDetection tag : tags) 
                draw(mat, tag);

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
        double[] corners = tag.getCorners();
        Point prev = new Point(corners[7], corners[8]);

        for (int ind = 0; ind < 8; ind+=2) {
            Point cur = new Point(corners[ind], corners[ind+1]);
            line(mat, prev, cur, Scalar(255, 0, 0), 3);
            prev = cur;
        }
    }
    
}
