package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name = "Madagas Tele", group = "!")
public class PenguinTele extends H2OLooTeleOp {

    Servo servo1;

    boolean isAPressed = false;

    enum dPad{
        RIGHT,
        LEFT,
        UP,
        DOWN
    }

    dPad last_dpad;

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);
        last_dpad = dPad.RIGHT;

    }

    @Override
    public void opModePeriodic() {



        if (last_dpad == dPad.RIGHT)
        {
            driveTrain.teleOpDrive(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    attachmentControl
            );
        }
        if (last_dpad == dPad.LEFT)
        {
            driveTrain.teleOpDrive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    attachmentControl
            );
        }
        if (last_dpad == dPad.UP)
        {
            driveTrain.teleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    attachmentControl
            );
        }
        if (last_dpad == dPad.DOWN)
        {
            driveTrain.teleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    attachmentControl
            );
        }

        if (gamepad1.dpad_right)
        {
            last_dpad = dPad.RIGHT;
        }
        if (gamepad1.dpad_left)
        {
            last_dpad = dPad.LEFT;
        }
        if (gamepad1.dpad_up)
        {
            last_dpad = dPad.UP;
        }
        if (gamepad1.dpad_down)
        {
            last_dpad = dPad.DOWN;
        }

//        attachmentControl.droneTeleOp();
//        attachmentControl.hangServoTeleOp();
//        attachmentControl.hangMotorManual();
        attachmentControl.armTeleOp();
        attachmentControl.clawPickupTeleOp(gamepad1.x, gamepad1.b, gamepad1.a);

    }

}

