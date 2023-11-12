package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedPark_2ndtileOnly extends H2OLooAuto {

    public void opModeInit() {

//        while (opModeIsActive()) cameraControl.telemetryAprilTag();

    }

    public void opModePeriodic() {

        driveTrain.EncoderAutoMecanumDrive(24, 0, 0, 0.75, 3);
        driveTrain.EncoderAutoMecanumDrive(0, 0, 90, 0.75, 5);
        driveTrain.EncoderAutoMecanumDrive(84, 0, 0, 0.75, 8);
        attachmentControl.intakeAuto();
        driveTrain.EncoderAutoMecanumDrive(-6, 0, 0, 0.75, 1);
        sleep(1000);
        driveTrain.EncoderAutoMecanumDrive(6, 0, 0, 0.75, 1);
        sleep(10000);

//        attachmentControl.intakeAuto();

//        odometryControl.strafeLeft(24, 0.5);

//        while (opModeIsActive()) {
//
//            odometryControl.odoTelemetry(Encoder.MeasurementUnit.PULSES);
//            cameraControl.telemetryAprilTag();
//            telemetryControl.update();
//        }

    }

}
