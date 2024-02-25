package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Backstage", group = "BLUE")
public class BlueBackstage extends H2OLooAuto {

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
            // Backdrop side will be at higher motor speed so it is quicker to complete tasks to avoid collision.
            case RIGHT:
//                1. Reverse 2" to clear the rigging (just in case).
                driveTrain.EncoderAutoMecanumDrive(-2, 0, 0, 0.8, 1);
//                2. Perform a smooth/swing 50° CW/right turn as the robot strafe left for 32" at 80% speed. This drives pixel onto the spike mark.
                driveTrain.EncoderAutoMecanumDrive(0, -32, 50, 0.8, 2);
//                3. Strafe right 15" with 50°CCW turn to release/leave the purple pixel at the left spike mark with the rear facing the backboard.
                driveTrain.EncoderAutoMecanumDrive(0, 15, -50, 0.8, 2);
//                4. Strafe left 8" to line up robot roughly in the middle between the wingnuts holding the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, -8, 0, 0.8, 1);
//                5. Reverse 38" to hit and square the robot against the backdrop.
                driveTrain.EncoderAutoMecanumDrive(-38, 0, 0, 0.4, 3);
//                6. Drive forward 15" to provide space for the scoring arm.
                driveTrain.EncoderAutoMecanumDrive(15, 0, 0, 0.5, 2);
//                7. Strafe left 12" to align in front of RIGHT AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0, -12, 0, 0.5, 2);
//                ## The robot is in position to score.
                break;
            case CENTER:
//                1. Perform a diagonal strafe (3" reverse and 32" left) to clear the rigging and deliver the pixel.
                driveTrain.EncoderAutoMecanumDrive(-3, -32, 0, 0.8, 3);
//                2. Strafe right 3" (at 50% speed) to release/leave the purple pixel at the left spike mark. Robot is already aligned to CENTER AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.8, 1);
//                3. Reverse 38" to hit and square the robot against the backdrop.
                driveTrain.EncoderAutoMecanumDrive(-38, 0, 0, 0.4, 3);
//                4. Drive forward 17" to provide space for the scoring arm.
                driveTrain.EncoderAutoMecanumDrive(17, 0, 0, 0.5, 2);
//                ## The robot is in position to score.
                break;
            case LEFT:
//                1. Reverse 2" to clear the rigging (just in case).
                driveTrain.EncoderAutoMecanumDrive(-2, 0, 0, 0.8, 1);
//                2. Perform a diagonal strafe (12" reverse and 26" left) to deliver the pixel.
                driveTrain.EncoderAutoMecanumDrive(-12, -26, 0, 0.8, 2);
//                3. Strafe right 3" to align to LEFT AprilTag
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.8, 1);
//                4. Reverse 10" to get to the scoring zone prep position.
                driveTrain.EncoderAutoMecanumDrive(-10, 0, 0, 0.5, 1);
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
        driveTrain.EncoderAutoMecanumDrive(-5, 0, 0, 0.2, 3);
        // Still need to release the claw to drop the yellow pixel. Need to create separate function to open claw or add into scoreAuto.
        attachmentControl.dropYellowAuto();
        sleep(250);
        attachmentControl.resetArmAuto();
//        sleep();

        /*
        Parking path case statements.
        Objective is to park in A6 zone.
         */
        switch (location) {
            case RIGHT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, 39, 0, 0.8, 3);//34
                driveTrain.EncoderAutoMecanumDrive(-20, 0, 0, 0.8, 3);
                break;
            case CENTER:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, 30, 0, 0.8, 3);
                driveTrain.EncoderAutoMecanumDrive(-20, 0, 0, 0.8, 3);
                break;
            case LEFT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, 24, 0, 0.8, 3);
               driveTrain.EncoderAutoMecanumDrive(-20, 0, 0, 0.8, 3);
                break;
            case NONE:
                break;

        }

        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy()); // Keep robot operating until all motor functions are completed.

    }

}
