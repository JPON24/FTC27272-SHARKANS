package org.firstinspires.ftc.teamcode;

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
import java.util.List;



//@TeleOp(name = "OpenCV Testing")

public class opencv{
    double cX = 0;
    double cY = 0;
    double width = 0;
    double height = 0 ;

    int widthAvg = 0;
    int heightAvg = 0;

    double theta = 0;

    MatOfPoint bigcontour;

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

    public void init(HardwareMap hwMap)
    {
        initOpenCV(hwMap);
    }
//    @Override
//    public void runOpMode() {
//
//        initOpenCV();
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();
//
//
//            // The OpenCV pipeline automatically processes frames and handles detection
//        }
//
//        // Release resources
//        controlHubCam.stopStreaming();
//    }

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
                 boxH = 166
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

                // ( distW + distH + distA ) / 3


                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                String heightLabel = "Height: " + (int) height + " pixels";
                Imgproc.putText(input, heightLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

//                Display the Distance
                String distanceLabel = "Distance: " + GetDistance() + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 100), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);

//                double denominator = moments.get_m20() - moments.get_m02();
//                double numerator = 2 * moments.get_m11();
//                double radToDeg = 180/Math.PI;
//                double atanVal = Math.atan2(numerator,denominator);
//                atanVal /= 2;
//                theta = atanVal * radToDeg;
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                double initTheta = Math.atan2(height,width) * 180/Math.PI;
                initTheta -= 30;
                theta = initTheta * 3.85;

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
            Scalar lowerYellow = new Scalar(91, 52, 0);
            Scalar upperYellow = new Scalar(114, 255, 255);

            Scalar lowerBlue = new Scalar(0,0,0);
            Scalar upperBlue = new Scalar(20,255,255);

            Scalar lowerRed = new Scalar(112,70,0);
            Scalar upperRed = new Scalar(154,254,255);

            Mat mask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            return mask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            bigcontour = largestContour;
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

    public MatOfPoint GetBigContour()
    {
        return bigcontour;
    }

    public double GetDistance()
    {
        double distW = objW * focalLengthW / width;
        double distH = objW * focalLengthH / height;
        double distA = k/Math.sqrt(width * height);
        return ( distW + distH + distA ) / 3;
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