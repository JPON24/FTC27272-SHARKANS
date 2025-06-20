package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.onbotjava.OnBotJavaWebInterfaceManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;



//@TeleOp(name = "OpenCV Testing")

public class opencv{
    double cX = 0;
    double cY = 0;
    double width = 0;
    double height = 0 ;

    double theta = 0;
    double wrappedTheta = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
//    public static final double objectWidthInRealWorldUnits = 3.5;  // Replace with the actual width of the object in real-world units
//    public static final double focalLength = 720;  // Replace with the focal length of the camera in pixels

    private final double objW = 3.5;
//    private final double objH = 1.5;

    private final double focalLengthW = 823;
    private final double focalLengthH = 500;
    private final double k = 2146;

    private Mat emptyMat = new Mat();
    List<Point> points;

    double xMin = 640;
    double xMax = 0;

    double yMin = 480;
    int yMinIndex = 0;
    double yMax = 0;
    int yMaxIndex = 0;

    double yMaxX = 0;

    double minDiff = 0;
    double maxDiff = 0;


    double focalLength = 4; // mm
    double realHeight = 88.9; //mm
    double realWidth = 38.1; //mm
    double sensorWidth = 3.58; //mm
    double sensorHeight = 2.02; //mm
    double imageWidth = 640; // px
    double imageHeight = 480; // px

    public void init(HardwareMap hwMap)
    {
        initOpenCV(hwMap);
    }

    private void initOpenCV(HardwareMap hwMap) {
        // Create an instance of the camera
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new BlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class BlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
//            Mat mask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
            Imgproc.findContours(preprocessFrame(input), contours, emptyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);
                height = calculateHeight(largestContour);

                /* realW = 3.5
                 realH = 1.5
                 distance = 12
                 distance = 12
                 boxW = 240
                 boxH = 166`
                 boxA = 200 * 160

                 focalLengthW = 823
                 focalLengthH = 1328
                 k = 2146
                 */

                // focalLengthPxW
                // distance * boxWidth / realW

                // focalLengthPxH
                // distance * boxHeight / realH

                // distInchesW
                // realWidth * focalLengthPxW / BoxW

                // distInchesH
                // realHeight * focalLengthPxH / boxHeight

                // k (area const)
                // dist * sqrt(boxW * boxH)

                // distA
                // k / sqrt(boxW * boxH)

                // turn angle
                // offsetPixels * (cameraFOV / imgWidth)

                // ( distW + distH + distA ) /


                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                String heightLabel = "Height: " + (int) height + " pixels";
                Imgproc.putText(input, heightLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                Display the Distance
                String distanceLabel = "Distance: " + GetDistance() + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 100), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);

                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                double numerator = 2 * moments.get_mu11();
                double denominator = moments.get_mu20() - moments.get_mu02();

                theta = Math.atan2(numerator,denominator) / 2;
                theta *= 180/Math.PI;

                if (theta > 0)
                {
                    theta = 90 - theta + 90;
                }
                theta = Math.abs(theta);

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // tuned values
            Scalar lowerYellow = new Scalar(78, 75, 95); // 91, 150, 0
            Scalar upperYellow = new Scalar(110, 255, 255); // 114, 255, 255

            Scalar lowerBlue = new Scalar(0,67,0);
            Scalar upperBlue = new Scalar(20,255,255);

            Scalar lowerRed = new Scalar(109,63,0);
            Scalar upperRed = new Scalar(143,254,255);

            Mat mask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            return mask;
        }

        private MatOfPoint findLargestContour(@NonNull List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

        private double calculateHeight(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.height;
        }
    }

    public void StartStream(Telemetry tel)
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tel = new MultipleTelemetry(tel, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
    }

    public void StopStream()
    {
        controlHubCam.stopStreaming();
    }

    public int GetCX()
    {
        return (int)cX;
    }

    public List<Point> GetPoints()
    {
        return points;
    }

    public double GetWrappedTheta()
    {
        return wrappedTheta;
    }

    public double GetMinDiff()
    {
        return minDiff;
    }

    public double GetMaxDiff()
    {
        return maxDiff;
    }

    public double GetYMin()
    {
        return yMin;
    }

    public double GetYMinX()
    {
        return yMaxX;
    }

    public double GetXMin()
    {
        return xMin;
    }

    public double GetXMax()
    {
        return xMax;
    }

    public double GetDistance()
    {
//        double distW = objW * focalLengthW / width;
//        double distH = objW * focalLengthH / height;
//        double distA = k/Math.sqrt(width * height);
//        return ( distW + distH + distA ) / 3;

        double distanceH = focalLength * realHeight * imageHeight / (height * sensorHeight);
        double distanceW = focalLength * realWidth * imageWidth / (width * sensorWidth);
        double average = (distanceH + distanceW) / 2;

        average /= 25.4;

//        double normTheta = 90 - (Math.abs(GetTheta()) % 90);
//
//        double angleDist = Math.abs(45 - normTheta);
//
//        double coef = 1 + 0.1538 * (1 - angleDist/45);

        double coef = 1;
        return average * coef;


//        double tempWidth = objectWidthInRealWorldUnits - (scalarConst * (GetTheta()/90));
//        return (objectWidthInRealWorldUnits * focalLength) / (width + theta);
//        return (objectWidthInRealWorldUnits * focalLength) / width;
    }

    private double LowPass(double average, double newValue) {
        average = (average * 0.85) + (0.15 * newValue);

        return average;
    }

    public double GetTheta()
    {
        return theta;
    }
}
