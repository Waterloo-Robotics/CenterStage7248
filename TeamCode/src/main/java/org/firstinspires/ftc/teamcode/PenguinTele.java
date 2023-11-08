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

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);

    }

    @Override
    public void opModePeriodic() {

        driveTrain.teleOpDrive(
                gamepad1.left_stick_y * 0.7,
                gamepad1.left_stick_x * 0.7,
                gamepad1.right_stick_x * 0.7
        );

        attachmentControl.droneTeleOp();
        attachmentControl.intakeTeleOp();
        attachmentControl.hangServoTeleOp();
        attachmentControl.hangMotorManual();

    }

}
