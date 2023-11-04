package org.firstinspires.ftc.teamcode.tests;

import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

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
        telemetryControl.addData("Image Analysis",pipeline.getAnalysis());
//        telemetryControl.addData();
        telemetryControl.update();
    }


}

class SamplePipeline extends OpenCvPipeline {

    Mat gray = new Mat();

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    Mat RectA_Y = new Mat();
    int avg;

    static final int WidthRectA = 100;
    static final int HeightRectA = 100;
    static final Point RectATopLeftAnchor = new Point(200, 150);
    Point RectATLCorner = new Point(
            RectATopLeftAnchor.x,
            RectATopLeftAnchor.y);
    Point RectABRCorner = new Point(
            RectATopLeftAnchor.x + WidthRectA,
            RectATopLeftAnchor.y + HeightRectA);

    void inputToY(Mat grey) {
        Imgproc.cvtColor(grey, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);

    }

    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
        RectA_Y = Y.submat(new Rect(RectATLCorner, RectABRCorner));
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        avg = (int) Core.mean(RectA_Y).val[0];
        YCrCb.release(); // don't leak memory!
        Y.release(); // don't leak memory!


        Imgproc.rectangle( // rings
                gray, // Buffer to draw on
                RectATLCorner, // First point which defines the rectangle
                RectABRCorner, // Second point which defines the rectangle
                new Scalar(0,0,255), // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        return gray;
    }

    public int getAnalysis() {
        return avg;
    }

}

//class SamplePipeline extends OpenCvPipeline {
//
//    Scalar RED = new Scalar(81, 240, 90);
//
//    Mat YCrCb = new Mat();
//    Mat Y = new Mat();
//    Mat RectA_Y = new Mat();
//    int avg;
//    int avgA;
//    static final int STREAM_WIDTH = 1920; // modify for your camera
//    static final int STREAM_HEIGHT = 1080; // modify for your camera
//
//
//    static final int WidthRectA = 500;
//    static final int HeightRectA = 500;
//
//
//    static final Point RectATopLeftAnchor = new Point((STREAM_WIDTH - WidthRectA) / 2 + 300, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);
//    Point RectATLCorner = new Point(
//            RectATopLeftAnchor.x,
//            RectATopLeftAnchor.y);
//    Point RectABRCorner = new Point(
//            RectATopLeftAnchor.x + WidthRectA,
//            RectATopLeftAnchor.y + HeightRectA);
//
//
//
//
//    /*
//     * This function takes the RGB frame, converts to YCrCb,
//     * and extracts the Y channel to the 'Y' variable
//     */
//    void inputToY(Mat input) {
//        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
//        Core.split(YCrCb, yCrCbChannels);
//        Y = yCrCbChannels.get(0);
//
//    }
//
//    @Override
//    public void init(Mat firstFrame) {
//        inputToY(firstFrame);
//        RectA_Y = Y.submat(new Rect(RectATLCorner, RectABRCorner));
//    }
//
//    @Override
//    public Mat processFrame(Mat input) {
//        inputToY(input);
////        System.out.println("processing requested");
//        avg = (int) Core.mean(Y).val[0];
//        avgA = (int) Core.mean(RectA_Y).val[0];
//        YCrCb.release(); // don't leak memory!
//        Y.release(); // don't leak memory!
//
//
//        Imgproc.rectangle( // rings
//                input, // Buffer to draw on
//                RectATLCorner, // First point which defines the rectangle
//                RectABRCorner, // Second point which defines the rectangle
//                new Scalar(0,0,255), // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines
//
//        return input;
//    }
//
//    public int getAnalysis() {
//        return avg;
//    }
//    public int getRectA_Analysis() {
//        return avgA;
//    }
//
//}
