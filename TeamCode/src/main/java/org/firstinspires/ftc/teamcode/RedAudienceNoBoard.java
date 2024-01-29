package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Backstage Park C6")
public class RedAudienceNoBoard extends H2OLooAuto {

    CameraControl.PropLocation location;

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);
        initCamera(CameraControl.Alliance.BLUE);

        while (opModeInInit()) {

            telemetryControl.addData("Prop Location", cameraControl.getLocation());
            telemetryControl.update();

        }

    }

    @Override
    public void opModePeriodic() {
        location = cameraControl.getLocation();
        cameraControl.close();

        switch (location) {

            // Webcam 2 (robot left side) is facing the prop with the purple pixel in the push slot.
            case RIGHT:
                /*
                1. Perform a smooth/swing 50° CCW/right turn as the robot strafe left for 29" at 50% speed.
                2. Strafe right 3" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                3. Perform a smooth/swing 50° CW turn while driving forward for 18" @ 50% speed.
                4. Perform a point turn 100° CCW to align the camera to AprilTag or robot claw in front of the left AprilTag.
                5. Drive/Reverse straight 19" to get the rear robot frame on the backstage park/score zone line.
                6. Strafe right 4" to align to Right AprilTag
                ## The robot is in position to score.
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -29, 50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-18, 0, 50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 0, -100, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-19, 0, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0,4,0,0.5,3);
                break;
            case CENTER:
                /*
                1. Strafe left 33" at 50% speed.
                2. Strafe right 3" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                3. Drive/Reverse straight 23" to get the rear robot frame on the backstage park/score zone line.
                ## The robot is in position to score.
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -33, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-23, 0, 0, 0.5, 3);
                break;
            case LEFT:
                /*
                1. Reverse by 2" to ensure we clear the rigging while making the turns.
                2. Perform a smooth/swing 50° CW/right turn as the robot strafe left for 29" at 50% speed.
                2. Strafe right 3" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                3. Perform a smooth/swing 50° CW turn while driving forward for 18" @ 50% speed.
                4. Perform a point turn 175° CW to align the camera to AprilTag or robot claw in front of the left AprilTag.
                5. Drive/Reverse straight 8" to get the rear robot frame on the backstage park/score zone line.
                6. Strafe right 2" to align to Left AprilTag
                ## The robot is in position to score.
                 */
                driveTrain.EncoderAutoMecanumDrive(-2, 0, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, -26, -50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-14, 0, 48, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-8, 0, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0,-8,0,0.5,3);
                break;
            case NONE:
                break;

        }
//        // Rotate scoring arm to intermediate scoring position. Arm not extended to reduce momentum and keep robot stable.
//        attachmentControl.intermediateAuto();
//        // Wait for robot to stabilize from the rotating arm momentum.
//        sleep(250);
//        // Extend scoring to reach the backdrop.
//        attachmentControl.scoreAuto();
//        // Waits until arm gets in position before executing the next step.
//        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());
//        sleep(250);
//        // Still need to move the robot backwards to land the claw onto the backdrop (slowly).
//        driveTrain.EncoderAutoMecanumDrive(-3, 0, 0, 0.2, 3);
//        // Still need to release the claw to drop the yellow pixel. Need to create separate function to open claw or add into scoreAuto.
//        attachmentControl.dropYellowAuto();
//        sleep(500);
//        attachmentControl.resetArmAuto();
//        sleep(1000);
//
//        /*
//        Parking path case statements.
//         */
//        switch (location) {
//            case RIGHT:
//                /*
//                1. Strafe right x" (at 80% speed) to clear the backdrop.
//                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
//                 */
//                driveTrain.EncoderAutoMecanumDrive(0, -16, 0, 0.8, 3);
//                driveTrain.EncoderAutoMecanumDrive(-20, 0, 0, 0.8, 3);
//                break;
//            case CENTER:
//                /*
//                1. Strafe right x" (at 80% speed) to clear the backdrop.
//                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
//                 */
//                driveTrain.EncoderAutoMecanumDrive(0, -25, 0, 0.8, 3);
//                driveTrain.EncoderAutoMecanumDrive(-20, 0, 0, 0.8, 3);
//                break;
//            case LEFT:
//                /*
//                1. Strafe right x" (at 80% speed) to clear the backdrop.
//                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
//                 */
//                driveTrain.EncoderAutoMecanumDrive(0, -34, 0, 0.8, 3);
//               driveTrain.EncoderAutoMecanumDrive(-20, 0, 0, 0.8, 3);
//                break;
//            case NONE:
//                break;
//
//        }
//
//        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy()); // Keep robot operating until all motor functions are completed.

    }

}
