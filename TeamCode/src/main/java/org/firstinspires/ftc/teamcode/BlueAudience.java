package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Audience", group = "BLUE")
public class BlueAudience extends H2OLooAuto {

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
//              1. Perform a smooth/swing 60° CCW/left turn as the robot strafe left for 28" at 50% speed. This drives pixel onto the spike mark.
                driveTrain.EncoderAutoMecanumDrive(0, -28, -60, 0.5, 3);
//              2. Back away from pixel to release/leave the purple pixel at the left spike mark by strafe right 6" (at 50% speed) .
                driveTrain.EncoderAutoMecanumDrive(0, 6, 0, 0.5, 1);
//              3. Move towards the center while turning to orient robot's rear towards backdrop by driving forward 28" while performing smooth/swing 58° CW turn.
                driveTrain.EncoderAutoMecanumDrive(28, 0, 58, 0.5, 3);
//              4. Strafe left 10" to land the robot on tile C2 (center blue side) with the rear facing the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, -10, 0, 0.5, 1);
//              5. Drive/Reverse straight 87" to get the rear robot frame on the backstage park/score zone line.
                driveTrain.EncoderAutoMecanumDrive(-87, 0, 0, 0.5, 6);
//              6. Strafe right 32" to align to LEFT AprilTag
                driveTrain.EncoderAutoMecanumDrive(0,32,0,0.5,2);
//              ## The robot is in position to score.
                break;
            case CENTER:
//                1. Strafe left 32" at 50% speed to deliver pixel.
                driveTrain.EncoderAutoMecanumDrive(0, -32, 0, 0.5, 3);
//                2. Strafe right 3" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.5, 3);
//                3. Drive forward 16" to get into tile B1 to clear purple pixel when driving towards tile C.
                driveTrain.EncoderAutoMecanumDrive(16, 0, 0, 0.5, 3);
//                4. Strafe left 24" to get to tile C1, and set to leave towards backdrop area.
                driveTrain.EncoderAutoMecanumDrive(0, -24, 0, 0.5, 3);
//                5. Reverse along column tiles C for 88". This will get the robot on the North end of tile C5.
                driveTrain.EncoderAutoMecanumDrive(-88, 0, 0, 0.5, 6);
//                6. Strafe right along row tiles 5 for 30" so the scoring claw is align with CENTER AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0, 30, 0, 0.5, 6);
//                ## The robot is in position to score.
                break;
            case RIGHT:
//                1. Perform a diagonal strafe (9" forward and 26" left) at 50% speed to deliver the pixel.
                driveTrain.EncoderAutoMecanumDrive(9, -26, 0, 0.5, 3);
//                2. Strafe right 22" (at 50% speed) to set robot to drive along column A (and under the rigging) towards the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 22, 0, 0.5, 3);
//                3. Drive 66" towards the backdrop along column A to land on the far side of tile A4.
                driveTrain.EncoderAutoMecanumDrive(-66, 0, 0, 0.5, 3);
//                4. Strafe left 35" and aligned in front of RIGHT AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0, -35, 0, 0.5, 3);
//                5. Drive/Reverse straight 12" to get the rear robot frame on the backstage park/score zone line.
                driveTrain.EncoderAutoMecanumDrive(-12,0,0,0.5,1);
//                ## The robot is in position to score.
                break;
            case NONE:
                break;

        }
        attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, gamepad1, gamepad2);
        // Rotate scoring arm to intermediate scoring position. Arm not extended to reduce momentum and keep robot stable.
        attachmentControl.intermediateAuto();
        // Wait for robot to stabilize from the rotating arm momentum.
        sleep(250);
        // Extend scoring to reach the backdrop.
        attachmentControl.scoreAuto();
        // Waits until arm gets in position before executing the next step.
        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());
        sleep(250);
        // Still need to move the robot backwards to land the claw onto the backdrop (slowly).
        driveTrain.EncoderAutoMecanumDrive(-4, 0, 0, 0.2, 3);
        // Still need to release the claw to drop the yellow pixel. Need to create separate function to open claw or add into scoreAuto.
        attachmentControl.dropYellowAuto();
        sleep(500);
        attachmentControl.resetArmAuto();
        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());
        sleep(1000);

        /*
        Parking path case statements.
        Objective is to park in C6 zone.
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
