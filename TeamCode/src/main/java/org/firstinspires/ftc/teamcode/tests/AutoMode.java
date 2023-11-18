package org.firstinspires.ftc.teamcode.tests;

import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@TeleOp(name="Auto: SkyStone Detector")
public class AutoMode extends LinearOpMode {
    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 480; // modify for your camera
    // Handle hardware stuff...

    TelemetryControl telemetryControl;
    int width = 640;
    int height = 480;
    // store as variable here so we can access the location
    EasyOpenCV detector = new EasyOpenCV(width);
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // robot logic...

        telemetryControl = new TelemetryControl(telemetry);
        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        webcam.setPipeline(detector);
        // Remember to change the camera rotation
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });
        telemetryControl.startCameraStream(webcam, 60);

        //...

        EasyOpenCV.PropLocation location = detector.getLocation();
        if (location != EasyOpenCV.PropLocation.NONE) {
            // Move to the left / right
        } else {
            // Grab the skystone
        }

        waitForStart();

        while (opModeIsActive()) {

            telemetryControl.startCameraStream(webcam, 60);
            telemetryControl.addData("Prop Position", detector.getLocation());
            telemetryControl.update();

        }

        // more robot logic...
    }

}

class EasyOpenCV extends OpenCvPipeline {

    enum PropLocation {
        LEFT,
        RIGHT,
        NONE
    }

    private int width; // width of the image
    PropLocation location;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public EasyOpenCV(int width) {
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
            location = PropLocation.NONE;
            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar upperBlue = new Scalar(139, 255, 255);
        Scalar lowerBlue = new Scalar(98, 50, 50);
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
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                left = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                right = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(input, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
        if (!left) location = PropLocation.LEFT;
        else if (!right) location = PropLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = PropLocation.NONE;

        return input; // return the mat with rectangles drawn
    }

    public PropLocation getLocation() {
        return this.location;
    }

}
