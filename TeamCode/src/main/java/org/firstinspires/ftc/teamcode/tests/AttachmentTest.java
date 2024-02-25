package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.H2OLooTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//@Disabled
@Config
@TeleOp(name = "Testimg l a la", group = "!")
public class AttachmentTest extends H2OLooTeleOp {

    public static int DESIRED_TAG_ID = 2;
    boolean isBPressed = false;

    public void opModeInit() {

        cameraControl = new CameraControl(hardwareMap, telemetryControl, CameraControl.Alliance.RED, gamepad1, gamepad2);
        cameraControl.initAprilTag(hardwareMap);
        cameraControl.setDesiredTagId(DESIRED_TAG_ID);

    }

    public void opModePeriodic() {
//        cameraControl.setDesiredTagId(DESIRED_TAG_ID);

//        attachmentControl.hangMotorManual();
//        attachmentControl.hangServoManual();
//
//
//        attachmentControl.hangServoManual();
//        attachmentControl.hangServoTeleOp();
//        attachmentControl.hangMotorManual(); // gamepad 2 left stick Y
        attachmentControl.extendArmMotorManual(); // gamepad 2 left stick y
        attachmentControl.rotateArmMotorManual(); // gamepad 2 right stick y
        attachmentControl.extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        attachmentControl.extendArmMotor.setTargetPositionTolerance(70);
        attachmentControl.rotateArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        attachmentControl.rotateArmMotor.setTargetPositionTolerance(70);
        attachmentControl.rotateArmMotor.setTargetPosition(1368); // original number 4900
        attachmentControl.extendArmMotor.setPower(1);
        attachmentControl.extendArmMotor.setTargetPosition(-1100); // original number -1255
        attachmentControl.rotateArmMotor.setPower(1);
        attachmentControl.clawRotate.setPosition(0.26);


        attachmentControl.clawPickupTest(gamepad2.x, gamepad2.b, gamepad2.a);
//        attachmentControl.armTeleOp();

        attachmentControl.clawRotateManual(); // gamepad 2 bumpers
//        attachmentControl.touchSensorTelemetry();
//
//        cameraControl.followAprilTag(driveTrain, attachmentControl);
//        attachmentControl.droneTeleOp();
//        driveTrain.teleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, attachmentControl);

    }

}
