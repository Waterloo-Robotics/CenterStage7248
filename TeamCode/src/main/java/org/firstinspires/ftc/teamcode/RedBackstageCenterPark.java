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
                driveTrain.EncoderAutoMecanumDrive(0, -29, -50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(18, 0, 50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 0, 175, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-12, 0, 0, 0.5, 3);
                // Webcam 2 (robot left side) is facing the prop with the purple pixel in the push slot.
//                // drive forward (strafe left) to the center of the second tile. 20" @ 50% power.
//                driveTrain.EncoderAutoMecanumDrive(0, -20, 0, 0.5, 3);
//                // spin turn 90° CCW to have the purple pixel facing the team prop. -90 @ 20% power
//                driveTrain.EncoderAutoMecanumDrive(0, 0, -90, 0.2, 3);
//                // drive forward (strafe left) to the LEFT "spike mark" to deliver the purple pixel. 3" @ 50% power
//                driveTrain.EncoderAutoMecanumDrive(0, -3, 0, 0.5, 3);
//                // reverse (strafe right) from the "spike mark" location to clear the purple pixel from the robot push frame. 5" strafe right @ 50% power.
//                driveTrain.EncoderAutoMecanumDrive(0, 5, 0, 0.5, 3);
//                // spin another 90°CCW to have the rear side (scoring side) facing the backdrop. -90 @ 20% power.
//                driveTrain.EncoderAutoMecanumDrive(0, 0, -90, 0.2, 3);
//                //strafe right a little bit to line the robot/claw to the LEFT April Tag ID. 3" strafe right @ 20% power.
//                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.2, 3);
                break;
            case RIGHT:
                // Webcam 2 (robot left side) is facing the prop with the purple pixel in the push slot.
                // drive forward (strafe left) to the center of the second tile. 20" @ 50% power.
                driveTrain.EncoderAutoMecanumDrive(0, -20, 0, 0.5, 3);
                // spin turn 90° CW to have the purple pixel facing the team prop. 90 @ 20% power
                driveTrain.EncoderAutoMecanumDrive(0, 0, 90, 0.2, 3);
                // drive forward (strafe left) to the RIGHT "spike mark" to deliver the purple pixel. 3" @ 50% power
                driveTrain.EncoderAutoMecanumDrive(0, -3, 0, 0.5, 3);
                // reverse (strafe right) from the "spike mark" location to clear the purple pixel from the robot push frame. 5" strafe right @ 50% power.
                driveTrain.EncoderAutoMecanumDrive(0, 5, 0, 0.5, 3);
                // spin another 90°CCW to have the rear side (scoring side) facing the backdrop. -90 @ 20% power.
                driveTrain.EncoderAutoMecanumDrive(0, 0, -90, 0.2, 3);
                //strafe left and then backwards to avoid the purple pixel just delivered. 5" strafe left @ 50% power.
                driveTrain.EncoderAutoMecanumDrive(0, -5, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-10, 0, 0, 0.5, 3);
                //strafe right a little bit to line the robot/claw to the RIGHT April Tag ID. 3" strafe right @ 20% power.
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.2, 3);
                break;
            case CENTER:
                // Webcam 2 (robot left side) is facing the prop with the purple pixel in the push slot.
                // drive forward (strafe left) to deliver the purple pixel to the CENTER "spike mark". 26" @ 50% power.
                driveTrain.EncoderAutoMecanumDrive(0, -26, 0, 0.5, 3);
                // reverse (strafe right) from the "spike mark" location to clear the purple pixel from the robot push frame. 5" strafe right @ 50% power.
                driveTrain.EncoderAutoMecanumDrive(0, 5, 0, 0.5, 3);
                // spin another 180°CCW to have the rear side (scoring side) facing the backdrop. -180 @ 20% power.
                driveTrain.EncoderAutoMecanumDrive(0, 0, -180, 0.2, 3);
                //strafe left a little bit to line the robot/claw to the CENTER April Tag ID. 3" strafe right @ 20% power.
                driveTrain.EncoderAutoMecanumDrive(0, -3, 0, 0.2, 3);
                break;
            case NONE:
                break;

        }
        // Rotate scoring arm to intermediate scoring position. Arm not extended to reduce momentum and keep robot stable.
        attachmentControl.intermediateAuto();
        // Wait for robot to stabilize from the rotating arm momentum.
        sleep(250);
        // Extend scoring to reach the backdrop.
        attachmentControl.scoreAuto();
        // Waits until arm gets in position before executing the next step.
        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());
        // Still need to move the robot backwards to land the claw onto the backdrop (slowly).
        driveTrain.EncoderAutoMecanumDrive(-3, 0, 0, 0.2, 3);
        // Still need to release the claw to drop the yellow pixel. Need to create separate function to open claw or add into scoreAuto.
        attachmentControl.dropYellowAuto();
        sleep(1000);

        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy()); //What is this for?

    }

}
