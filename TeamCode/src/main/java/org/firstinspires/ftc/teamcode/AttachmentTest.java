package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.H2OLooTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AttachmentTest extends H2OLooTeleOp {

    boolean isBPressed = false;

    public void opModeInit() {}

    public void opModePeriodic() {

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

        driveTrain.teleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

    }

}
