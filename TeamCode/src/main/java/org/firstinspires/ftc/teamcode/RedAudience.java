package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Red Audience", group = "RED")
public class RedAudience extends H2OLooAuto {

    CameraControl.PropLocation location;

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);
        initCamera(CameraControl.Alliance.RED);

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
//                1. Perform a diagonal strafe (12" reverse and 30" left) to deliver the pixel.
                driveTrain.EncoderAutoMecanumDrive(-12, -30, 0, 0.5, 4);
//                2. Strafe right 26" (at 50% speed) to set robot to drive along column F (and under the rigging) towards the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 26, 0, 0.5, 3);
//                3. Drive 66" towards the backdrop along column A to land on the far side of tile F4.
                driveTrain.EncoderAutoMecanumDrive(66, 0, 0, 0.5, 5);
//                4. Strafe left 37" to align to LEFT AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0, -37, 0, 0.5, 3);
//                5. Rotate the robot 175°CCW so the rear faces the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 0, -175, 0.3, 3);
//                6. Reverse 9" to get into the scoring prep zone.
                driveTrain.EncoderAutoMecanumDrive(-9, 0, 0, 0.5, 2);
//                ## The robot is in position to score.
                break;
            case CENTER:
//                1. Perform a diagonal strafe (3" reverse and 32" left) to clear the rigging and deliver the pixel.
                driveTrain.EncoderAutoMecanumDrive(-3, -32, 0, 0.5, 2);
//                2. Strafe right 16" to leave the  pixel at the spike mark, and enough space to rotate robot.
                driveTrain.EncoderAutoMecanumDrive(0, 16, 0, 0.5, 2);
//                3. Rotate the robot 175°CW so the rear faces the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 0, 175, 0.5, 2);
//                4. Drive forward 26" to get in between E1 and F1, and hit the wall to square the robot.
                driveTrain.EncoderAutoMecanumDrive(26, 0, 0, 0.3, 3);
//                5. Reverse 8" to clear the white pixel towers when the robot strafe towards D1.
                driveTrain.EncoderAutoMecanumDrive(-8, 0, 0, 0.5, 1);
//                6. Strafe right 39" along row tiles 1. Robot should be in D1.
                driveTrain.EncoderAutoMecanumDrive(0, 39, 0, 0.5, 2);
//                7. Drive/Reverse straight 85" to get the rear robot frame on the backstage park/score zone line.
                driveTrain.EncoderAutoMecanumDrive(-85, 0, 0, 0.5, 6);
//              6. Strafe left 32" to align to CENTER AprilTag
                driveTrain.EncoderAutoMecanumDrive(0, -28, 0, 0.5, 3);
//                ## The robot is in position to score.
                break;
            case RIGHT:
//                1. Perform a smooth/swing 45° CW/right turn as the robot strafe left for 32" at 50% speed.
                driveTrain.EncoderAutoMecanumDrive(0, -32, 45, 0.5, 2);
//                2. Strafe right 6" (at 50% speed) to release/leave the purple pixel at the left spike mark.
                driveTrain.EncoderAutoMecanumDrive(0, 6, 0, 0.5, 1);
//                3. Reverse 24" towards Y1 intersection.
                driveTrain.EncoderAutoMecanumDrive(-24, 0, 0, 0.5, 2);
//                4. Perform a point turn 130° CW to so the rear faces the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 0, 130, 0.3, 4);
//                5. Strafe right 8" to place robot in between tiles D1 and D2.
                driveTrain.EncoderAutoMecanumDrive(0, 8, 0, 0.5, 1);
//                6. Drive/reverse 85" along column D to get the rear robot frame on the backstage park/score zone line.
                driveTrain.EncoderAutoMecanumDrive(-84,0,0,0.5,6);
//                7. Strafe left 32" to align to RIGHT AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0,-36,0,0.5,3);
//                ## The robot is in position to score.
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
        driveTrain.EncoderAutoMecanumDrive(-4, 0, 0, 0.2, 3);
        // Still need to release the claw to drop the yellow pixel. Need to create separate function to open claw or add into scoreAuto.
        attachmentControl.dropYellowAuto();
        sleep(500);
        attachmentControl.resetArmAuto();
        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy());
        sleep(1000);

        /*
        Parking path case statements.
        Objective is to park in D6 zone.
         */
        switch (location) {
            case RIGHT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, 38, 0, 0.8, 3);
                driveTrain.EncoderAutoMecanumDrive(-6, 0, 0, 0.8, 3);
                break;
            case CENTER:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, 30, 0, 0.8, 3);
                driveTrain.EncoderAutoMecanumDrive(-8, 0, 0, 0.8, 3);
                break;
            case LEFT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, 18, 0, 0.8, 3);
                driveTrain.EncoderAutoMecanumDrive(-8, 0, 0, 0.8, 3);
                break;
            case NONE:
                break;

        }

        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy()); // Keep robot operating until all motor functions are completed.

    }

}
