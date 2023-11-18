package org.firstinspires.ftc.teamcode.tests;

import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

//@Disabled
@TeleOp(name = "Blinded by the light", group = "!")
public class sampleCVPatrick extends OpMode {

    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 480; // modify for your camera
    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    TelemetryControl telemetryControl;

    @Override
    public void init() {
        telemetryControl = new TelemetryControl(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SamplePipeline();
        telemetryControl.startCameraStream(webcam, 60);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetryControl.addData("Camera Failed", "");
                telemetryControl.update();
            }
        });

    }

    @Override
    public void loop() {
        telemetryControl.startCameraStream(webcam, 60);
        telemetryControl.update();
    }
}

//class SamplePipeline extends OpenCvPipeline {
//
//    Mat hsv = new Mat();
//    Mat hsvr = new Mat();
//
//    Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
//    Core.inRange(hsv, new Scalar(98, 50, 50), new Scalar(200, 255, 255), hsvr);
//
//    Core.bitwise_and(hsv, hsv, hsvr)
//
//
//
////    Mat RectA_Y = new Mat();
//
////    static final int WidthRectA = 100;
////    static final int HeightRectA = 100;
////    static final Point RectATopLeftAnchor = new Point(200, 150);
////    Point RectATLCorner = new Point(
////            RectATopLeftAnchor.x,
////            RectATopLeftAnchor.y);
////    Point RectABRCorner = new Point(
////            RectATopLeftAnchor.x + WidthRectA,
////            RectATopLeftAnchor.y + HeightRectA);
//
//
//
//    @Override
//    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
//        Core.inRange(hsv, new Scalar(98, 50, 50), new Scalar(200, 255, 255), hsvr);
//
//        Core.bitwise_and(hsv, hsv, hsvr);
//
////        Imgproc.rectangle( // rings
////                hsv, // Buffer to draw on
////                RectATLCorner, // First point which defines the rectangle
////                RectABRCorner, // Second point which defines the rectangle
////                new Scalar(0, 0, 255), // The color the rectangle is drawn in
////                2); // Thickness of the rectangle lines
//
//        return hsvr;
//    }


