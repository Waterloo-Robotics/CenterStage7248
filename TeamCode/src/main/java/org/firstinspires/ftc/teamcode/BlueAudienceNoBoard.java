package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Audience No Board")
public class BlueAudienceNoBoard extends H2OLooAuto {

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
            // Review Appendix B of game manual 2 Appendix B (page 38) to find out playing field coordinates.
            case LEFT:
                /*
                1. Perform a smooth/swing 50° CCW/left turn as the robot strafe left for 29" at 50% speed.
                2. Strafe right 3" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                3. Perform a smooth/swing 47° CW turn while driving forward for 18" @ 50% speed. Move further away from pixel while trying to square the rear of the robot to face the backdrop.
                4. The orientation of the robot should now be the same as the starting point. Strafe left 20" to land the robot on tile C2 (center blue side).
                5. Drive/Reverse straight 81" to get the rear robot frame on the backstage park/score zone line.
                6. Strafe right 36" to align to Left AprilTag
                ## The robot is in position to score.
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -28, -50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(18, 0, 45, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, -20, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-81, 0, 0, 0.5, 6);
                driveTrain.EncoderAutoMecanumDrive(0,30,0,0.5,3);
                break;
            case CENTER:
                /*
                1. Strafe left 32" at 50% speed to deliver pixel.
                2. Strafe right 3" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                3. Drive forward 16" to get into tile B1 to clear purple pixel when driving towards tile C.
                4. Strafe left 25" to get to tile C1, and set to leave towards backdrop area.
                5. Reverse along column tiles C for 86". This will get the robot on the North end of tile C5.
                6. Strafe right along row tiles 5 for 30" so the scoring claw is align with center AprilTag.
                ## The robot is in position to score.
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -32, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(16, 0, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, -25, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-86, 0, 0, 0.5, 6);
                driveTrain.EncoderAutoMecanumDrive(0, 27, 0, 0.5, 6);
                break;
            case RIGHT:
                /*
                1. Perform a smooth/swing 50° CW/right turn as the robot strafe left for 29" at 50% speed.
                2. Strafe right 6" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                3. Pivot turn CW 39° so the camera is facing (square) the audience.
                4. Perform a point turn 175° CW to align the camera to AprilTag or robot claw in front of the left AprilTag.
                5. Drive/Reverse straight 8" to get the rear robot frame on the backstage park/score zone line.
                6. Strafe right 2" to align to Left AprilTag
                ## The robot is in position to score.
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -25, 50, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 6, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 0, 39, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-30, 0, 0, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(0, 0, -86, 0.5, 3);
                driveTrain.EncoderAutoMecanumDrive(-68,0,0,0.5,6);
                driveTrain.EncoderAutoMecanumDrive(0,22,0,0.5,3);
                break;
            case NONE:
                break;

        }
        attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, gamepad1, gamepad2);
        // Rotate scoring arm to intermediate scoring position. Arm not extended to reduce momentum and keep robot stable.
        attachmentControl.intermediateAuto();
//        // Wait for robot to stabilize from the rotating arm momentum.
        sleep(250);
//        // Extend scoring to reach the backdrop.
        attachmentControl.scoreAuto();
//        // Waits until arm gets in position before executing the next step.
        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());
        sleep(250);
        // Still need to move the robot backwards to land the claw onto the backdrop (slowly).
        driveTrain.EncoderAutoMecanumDrive(-6, 0, 0, 0.2, 3);
        // Still need to release the claw to drop the yellow pixel. Need to create separate function to open claw or add into scoreAuto.
        attachmentControl.dropYellowAuto();
        sleep(500);
        attachmentControl.resetArmAuto();
        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());
        sleep(1000);

        /*
        Parking path case statements.
         */
        switch (location) {
            case RIGHT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -10, 0, 0.8, 3);
                driveTrain.EncoderAutoMecanumDrive(-6, 0, 0, 0.8, 3);
                break;
            case CENTER:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -20, 0, 0.8, 3);
                driveTrain.EncoderAutoMecanumDrive(-8, 0, 0, 0.8, 3);
                break;
            case LEFT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -30, 0, 0.8, 3);
               driveTrain.EncoderAutoMecanumDrive(-8, 0, 0, 0.8, 3);
                break;
            case NONE:
                break;

        }

        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy()); // Keep robot operating until all motor functions are completed.

    }

}
