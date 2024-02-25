package org.firstinspires.ftc.teamcode.tests;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooTeleOp;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp(name = "Test Encoders", group = "TEST")
public class EncoderTest extends OpMode {

    TelemetryControl telemetryControl;
    DriveTrain driveTrain;
    @Override
    public void init() {

        telemetryControl = new TelemetryControl(telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetryControl, DcMotor.ZeroPowerBehavior.FLOAT, gamepad1, gamepad2);

    }

    @Override
    public void loop() {

        telemetryControl.addData("Front Left Position", driveTrain.fl.getCurrentPosition());
        telemetryControl.addData("Front Right Position", driveTrain.fr.getCurrentPosition());
        telemetryControl.addData("Back Left Position", driveTrain.bl.getCurrentPosition());
        telemetryControl.addData("Back Right Position", driveTrain.br.getCurrentPosition());
        telemetryControl.update();

    }

}
