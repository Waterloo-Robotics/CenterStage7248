package org.firstinspires.ftc.robotcontroller.H2OLoo.samples;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Mecanum TeleOp", group = "H2OLoo Samples")
public class MecanumDrive extends H2OLooTeleOp {

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);

    }

    @Override
    public void opModePeriodic() {

        while (opModeIsActive()) {

            driveTrain.teleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    attachmentControl
            );

        }

    }

}
