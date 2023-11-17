package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.tests.sampleCV.STREAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.tests.sampleCV.STREAM_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
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

@TeleOp(name = "Light of my Life", group = "!")
public class sampleCV extends OpMode {
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
        webcam.setPipeline(pipeline);
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
        FtcDashboard.getInstance().startCameraStream(webcam,60);
    }

    @Override
    public void loop() {

    }


}

class SamplePipeline extends OpenCvPipeline {
    private double borderLeftX = 0;     //fraction of pixels from the left side of the cam to skip
    private double borderRightX = 0;    //fraction of pixels from the right of the cam to skip
    private double borderTopY = 0;      //fraction of pixels from the top of the cam to skip
    private double borderBottomY = 0;   //fraction of pixels from the bottom of the cam to skip

    private int loopCounter = 0;
    private int pLoopCounter = 0;

    private final Mat mat = new Mat();
    private final Mat processed = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);

    private double maxArea = 0;
    private boolean first = false;



//    void inputToY(Mat input) {
//
//        Mat blurredImage = new Mat();
//        Mat hsvImage = new Mat();
//        Mat mask = new Mat();
//        Mat morphOutput = new Mat();
//
//        Imgproc.blur(input, blurredImage, new Size(15, 15));
//        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
//
//    }

    @Override
    public void init(Mat firstFrame) {
        processFrame(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        Scalar upperBlue = new Scalar(139, 255, 255);
        Scalar lowerBlue = new Scalar(98, 50, 50);
        Scalar upperRed = new Scalar(30, 255, 255);
        Scalar lowerRed = new Scalar(0, 50, 50);

        Mat into_hsv = new Mat();
        Imgproc.cvtColor(input, into_hsv, Imgproc.COLOR_BGR2HSV);

        Mat processed = new Mat();
        Core.inRange(into_hsv, lowerRed, upperRed, processed);
//        Core.inRange(into_hsv, lowerRed, upperRed, processed);
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

        Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.0);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

        int x = 0;
        int y = 0;
        int h = (int) processed.size().height;
        int w = (int) (processed.size().width / 3.0);

        Imgproc.rectangle(
                input,
                new Rect(
                        x,
                        y,
                        w,
                        h
                ),
                new Scalar(196, 23, 112),
                2
        );

        Imgproc.rectangle(
                input,
                new Rect(
                        x + 2 * w,
                        y,
                        w,
                        h
                ),
                new Scalar(196, 23, 112),
                2
        );

        for (MatOfPoint contour : contours) {
            Point[] contourArray = contour.toArray();

            // Bound Rectangle if Contour is Large Enough
            if (contourArray.length >= 15) {
                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                Rect rect = Imgproc.boundingRect(areaPoints);

                if (                        rect.area() > maxArea
                        && rect.x + (rect.width / 2.0)  > (borderLeftX * STREAM_WIDTH)
                        && rect.x + (rect.width / 2.0)  < STREAM_WIDTH - (borderRightX * STREAM_WIDTH)
                        && rect.y + (rect.height / 2.0) > (borderTopY * STREAM_HEIGHT)
                        && rect.y + (rect.height / 2.0) < STREAM_HEIGHT - (borderBottomY * STREAM_HEIGHT)

                        || loopCounter - pLoopCounter   > 6
                        && rect.x + (rect.width / 2.0)  > (borderLeftX * STREAM_WIDTH)
                        && rect.x + (rect.width / 2.0)  < STREAM_WIDTH - (borderRightX * STREAM_WIDTH)
                        && rect.y + (rect.height / 2.0) > (borderTopY * STREAM_HEIGHT)
                        && rect.y + (rect.height / 2.0) < STREAM_HEIGHT - (borderBottomY * STREAM_HEIGHT)
                ){
                    maxArea = rect.area();
                    maxRect = rect;
                    pLoopCounter++;
                    loopCounter = pLoopCounter;
                    first = true;
                }
                else if(loopCounter - pLoopCounter > 10){
                    maxArea = new Rect().area();
                    maxRect = new Rect();
                }

                areaPoints.release();
            }
            contour.release();
        }
        if (contours.isEmpty()) {
            maxRect = new Rect(600,1,1,1);
        }


        return input;
    }
}


