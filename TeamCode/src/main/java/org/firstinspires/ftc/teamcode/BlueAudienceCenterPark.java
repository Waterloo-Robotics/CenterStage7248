package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Audience Center Park", group = "Blue!")
public class BlueAudienceCenterPark extends H2OLooAuto {

    public void opModeInit() {

//        while (opModeIsActive()) cameraControl.telemetryAprilTag();

    }

    public void opModePeriodic() {

        driveTrain.EncoderAutoMecanumDrive(27, 0, 0, 0.75, 3); //26 (changing because we run over the pixel)
        sleep(250);
        driveTrain.EncoderAutoMecanumDrive(-5, 0, 0, 0.75, 3); //-4
        sleep(250);
        driveTrain.EncoderAutoMecanumDrive(0, 0, -90, 0.75, 3);
        sleep(250);
        driveTrain.EncoderAutoMecanumDrive(0, 1, 0, 0.75, 5);
        sleep(250);
        driveTrain.EncoderAutoMecanumDrive(77, 0, 0, 0.75, 5);
        sleep(250);
//        attachmentControl.intakeAutoWithPower(-0.20);
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
