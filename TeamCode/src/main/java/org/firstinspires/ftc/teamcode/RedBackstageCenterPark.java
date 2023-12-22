package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Red Backstage Center Park", group = "Red!")
public class RedBackstageCenterPark extends H2OLooAuto {

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);

    }

    @Override
    public void opModePeriodic() {

        driveTrain.EncoderAutoMecanumDrive(26, 0, 0, 0.75, 3); //36, 0, 0, 0.75, 3
        sleep(250);
        driveTrain.EncoderAutoMecanumDrive(-6, 0, 0, 0.75, 1);
        sleep(250);
        driveTrain.EncoderAutoMecanumDrive(0,0, 90,0.75,1);
        sleep(250);
        driveTrain.EncoderAutoMecanumDrive(26, 0, 0, 0.75, 1);
//        attachmentControl.intakeAutoWithPower(-0.20);
        sleep(10000);

    }

}
