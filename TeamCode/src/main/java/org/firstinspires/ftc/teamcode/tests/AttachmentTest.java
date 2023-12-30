package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.H2OLooTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//@Disabled
@Config
@TeleOp
public class AttachmentTest extends H2OLooTeleOp {

    public static int DESIRED_TAG_ID = 2;
    boolean isBPressed = false;

    public void opModeInit() {

        cameraControl = new CameraControl(hardwareMap, telemetryControl, CameraControl.Alliance.RED, gamepad1, gamepad2);
        cameraControl.initAprilTag(hardwareMap);
        cameraControl.setDesiredTagId(DESIRED_TAG_ID);

    }

    public void opModePeriodic() {
        cameraControl.setDesiredTagId(DESIRED_TAG_ID);

//        if (gamepad1.b) {
//
//            if (!isBPressed) {
//
//                attachmentControl.droneTeleOp(true);
//
//            }
//
//            isBPressed = true;
//
//        } else {
//
//            isBPressed = false;
//
//        }

//        attachmentControl.hangMotorManual();
//        attachmentControl.hangServoManual();

//        attachmentControl.intakeManual(gamepad2.left_stick_x);

//        attachmentControl.hangServoManual();
//        attachmentControl.hangServoTeleOp();
//        attachmentControl.hangMotorManual(); // gamepad 2 left stick Y
        attachmentControl.extendArmMotorManual(); // gamepad 2 right stick x
        attachmentControl.rotateArmMotorManual(); // gamepad 2 right stick y
//        attachmentControl.armTeleOp();
//        attachmentControl.clawPickupTeleOp(gamepad2.x, gamepad2.b, gamepad2.a);
//        attachmentControl.clawRotateManual();
        attachmentControl.touchSensorTelemetry();

        cameraControl.followAprilTag(driveTrain, attachmentControl);
//        attachmentControl.droneTeleOp();
//        driveTrain.teleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, attachmentControl);

    }

}
