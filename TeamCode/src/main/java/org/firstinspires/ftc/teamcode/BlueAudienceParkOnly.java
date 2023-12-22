package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Audience Park Only", group = "Zoo!")
public class BlueAudienceParkOnly extends H2OLooAuto {

    public void opModeInit() {

//        while (opModeIsActive()) cameraControl.telemetryAprilTag();

    }

    public void opModePeriodic() {

        sleep(2000); // adding delay to avoid collision during match
        driveTrain.EncoderAutoMecanumDrive(4, 0, 0, 0.75, 3);
        driveTrain.EncoderAutoMecanumDrive(0, 0, -90, 0.75, 5);
        driveTrain.EncoderAutoMecanumDrive(77, 0, 0, 0.75, 8);
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
