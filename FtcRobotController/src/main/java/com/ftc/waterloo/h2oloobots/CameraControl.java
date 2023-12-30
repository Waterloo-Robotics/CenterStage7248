package com.ftc.waterloo.h2oloobots;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
public class CameraControl {

    TelemetryControl telemetryControl;
    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 480; // modify for your camera
    int width = 640;
    int height = 480;
    OpenCvWebcam openCvWebcam;
    BluePropPipeline bluePropPipeline;
    RedPropPipeline redPropPipeline;

    public enum Alliance {
        BLUE,
        RED
    }

    Alliance alliance = Alliance.RED;

    public enum PropLocation {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error. (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error. (0.25 / 25.0)
    final double TURN_GAIN   =  0.025  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    static int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)

    Gamepad gamepad1, gamepad2;


//    private AprilTagProcessor aprilTag;

//    private VisionPortal visionPortal;

    public CameraControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, @NonNull Alliance alliance, Gamepad gamepad1, Gamepad gamepad2) {

        this.telemetryControl = telemetryControl;
        this.alliance = alliance;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        openCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        bluePropPipeline = new BluePropPipeline(width);
        redPropPipeline = new RedPropPipeline(width);
        switch (alliance) {
            case RED:
                openCvWebcam.setPipeline(redPropPipeline);
                break;

            case BLUE:
                openCvWebcam.setPipeline(bluePropPipeline);
                break;
        }
        openCvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                openCvWebcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetryControl.addData("Camera Failed","");
                telemetryControl.update();
            }
        });
        FtcDashboard.getInstance().startCameraStream(openCvWebcam,60);

    }

    public void stream() {

        telemetryControl.startCameraStream(openCvWebcam, 60);

    }

    public void close() {

        openCvWebcam.closeCameraDevice();

    }

    public PropLocation getLocation() {

        PropLocation location;
        switch (alliance) {

            case BLUE:
                location = bluePropPipeline.getLocation();
                break;

            default:
            case RED:
                location = redPropPipeline.getLocation();
                break;

        }

        return location;
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag(HardwareMap hardwareMap) {

        this.close();
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    public void setDesiredTagId(int tagId) {

        DESIRED_TAG_ID = tagId;

    }

    public void followAprilTag(DriveTrain driveTrain, AttachmentControl attachmentControl) {
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null)
                    && ((DESIRED_TAG_ID <= 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetryControl.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetryControl.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetryControl.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetryControl.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetryControl.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetryControl.addData(">","Drive using joystick to find target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetryControl.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            telemetryControl.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }

        // Apply desired axes motions to the drivetrain.
        driveTrain.teleOpDrive(drive, strafe, turn, attachmentControl);
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.seconds() < 0.01);
    }

}

class RedPropPipeline extends OpenCvPipeline {

    private int width; // width of the image
    CameraControl.PropLocation location;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public RedPropPipeline(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            location = CameraControl.PropLocation.NONE;
            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar upperBlue = new Scalar(139, 255, 180);
        Scalar lowerBlue = new Scalar(98, 50, 25);
        Scalar upperRed = new Scalar(20, 255, 255);
        Scalar lowerRed = new Scalar(-5, 50, 75);
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowerRed, upperRed, thresh);
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, new Mat());

        Imgproc.GaussianBlur(thresh, thresh, new Size(5.0, 15.0), 0.0);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
//        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxVal = 0;
        int maxValIdx = 0;
        if (!contours.isEmpty()) {
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {

                double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }

            }
//        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));
            Imgproc.drawContours(input, contours, maxValIdx, new Scalar(255, 0, 0));

            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
//        for (int i = 0; i < contours.size(); i++) {
//            contoursPoly[i] = new MatOfPoint2f();
//            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
//            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
//        }

            contoursPoly[maxValIdx] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(maxValIdx).toArray()), contoursPoly[maxValIdx], 3, true);
            boundRect[maxValIdx] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[maxValIdx].toArray()));

            // Iterate and check whether the bounding boxes
            // cover left and/or right side of the image
            double left_x = 0.25 * width;
            double right_x = 0.825 * width;
            boolean left = false; // true if regular stone found on the left side
            boolean right = false; // "" "" on the right side
            boolean center = false;
//        for (int i = 0; i != boundRect.length; i++) {
//            if (boundRect[i].x < left_x)
//                left = true;
//            else if (boundRect[i].x + boundRect[i].width > right_x)
//                right = true;
//            else
//                center = true;
//
//            // draw red bounding rectangles on mat
//            // the mat has been converted to HSV so we need to use HSV as well
//            Imgproc.rectangle(input, boundRect[i], new Scalar(0.5, 76.9, 89.8));
//        }

            if (boundRect[maxValIdx].x < left_x)
                left = true;
            else if (boundRect[maxValIdx].x + boundRect[maxValIdx].width > right_x)
                right = true;
            else
                center = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(input, boundRect[maxValIdx], new Scalar(0.5, 76.9, 89.8));

            // if there is no yellow regions on a side
            // that side should be a Skystone
            if (left) location = CameraControl.PropLocation.LEFT;
            else if (right) location = CameraControl.PropLocation.RIGHT;
            else if (center) location = CameraControl.PropLocation.CENTER;
                // if both are true, then there's no Skystone in front.
                // since our team's camera can only detect two at a time
                // we will need to scan the next 2 stones
            else location = CameraControl.PropLocation.NONE;
        } else {
            location = CameraControl.PropLocation.CENTER;
        }

        return input; // return the mat with rectangles drawn
    }

    public CameraControl.PropLocation getLocation() {
        return this.location;
    }

}

class BluePropPipeline extends OpenCvPipeline {

    private int width; // width of the image
    CameraControl.PropLocation location;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public BluePropPipeline(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            location = CameraControl.PropLocation.NONE;
            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar upperBlue = new Scalar(139, 255, 180);
        Scalar lowerBlue = new Scalar(98, 50, 25);
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowerBlue, upperBlue, thresh);
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, new Mat());

        Imgproc.GaussianBlur(thresh, thresh, new Size(5.0, 15.0), 0.0);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
//        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxVal = 0;
        int maxValIdx = 0;

        if (!contours.isEmpty()) {
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {

                double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }

            }
//        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));
            Imgproc.drawContours(input, contours, maxValIdx, new Scalar(255, 0, 0));

            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
//        for (int i = 0; i < contours.size(); i++) {
//            contoursPoly[i] = new MatOfPoint2f();
//            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
//            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
//        }

            contoursPoly[maxValIdx] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(maxValIdx).toArray()), contoursPoly[maxValIdx], 3, true);
            boundRect[maxValIdx] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[maxValIdx].toArray()));

            // Iterate and check whether the bounding boxes
            // cover left and/or right side of the image
            double left_x = 0.25 * width;
            double right_x = 0.825 * width;
            boolean left = false; // true if regular stone found on the left side
            boolean right = false; // "" "" on the right side
            boolean center = false;
//        for (int i = 0; i != boundRect.length; i++) {
//            if (boundRect[i].x < left_x)
//                left = true;
//            else if (boundRect[i].x + boundRect[i].width > right_x)
//                right = true;
//            else
//                center = true;
//
//            // draw red bounding rectangles on mat
//            // the mat has been converted to HSV so we need to use HSV as well
//            Imgproc.rectangle(input, boundRect[i], new Scalar(0.5, 76.9, 89.8));
//        }

            if (boundRect[maxValIdx].x < left_x)
                left = true;
            else if (boundRect[maxValIdx].x + boundRect[maxValIdx].width > right_x)
                right = true;
            else
                center = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(input, boundRect[maxValIdx], new Scalar(0.5, 76.9, 89.8));

            // if there is no yellow regions on a side
            // that side should be a Skystone
            if (left) location = CameraControl.PropLocation.LEFT;
            else if (right) location = CameraControl.PropLocation.RIGHT;
            else if (center) location = CameraControl.PropLocation.CENTER;
                // if both are true, then there's no Skystone in front.
                // since our team's camera can only detect two at a time
                // we will need to scan the next 2 stones
            else location = CameraControl.PropLocation.NONE;
        } else {
            location = CameraControl.PropLocation.CENTER;
        }

        return input; // return the mat with rectangles drawn
    }

    public CameraControl.PropLocation getLocation() {
        return this.location;
    }

}
