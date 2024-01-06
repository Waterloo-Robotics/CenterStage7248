package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Red Backstage Center Park")
public class RedBackstageCenterPark extends H2OLooAuto {

    CameraControl.PropLocation location;

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);
        initCamera(CameraControl.Alliance.RED);

    }

    @Override
    public void opModePeriodic() {
        location = cameraControl.getLocation();
        cameraControl.close();

        switch (location) {

            case LEFT:
                driveTrain.EncoderAutoMecanumDrive(0, -27, -50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(18, 0, 50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 0, 175, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-24, 0, 0, 0.5, 3);
                break;

        }

        attachmentControl.intermediateAuto();
        sleep(500);
        attachmentControl.scoreAuto();
        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());

    }

}
