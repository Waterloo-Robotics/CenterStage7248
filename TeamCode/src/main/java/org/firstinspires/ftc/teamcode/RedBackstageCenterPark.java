package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Red Backstage Center Park")
public class RedBackstageCenterPark extends H2OLooAuto {

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);

    }

    @Override
    public void opModePeriodic() {

        driveTrain.EncoderAutoMecanumDrive(-26, 0, 0, 0.75, 3);
        driveTrain.EncoderAutoMecanumDrive(26, 0, 0, 0.75, 1);
        driveTrain.EncoderAutoMecanumDrive(0,-36, 0,0.75,1);

    }

}
