package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.AttachmentControl;
import com.ftc.waterloo.h2oloobots.CameraControl;
import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Red Backstage", group = "RED")
public class RedBackstage extends H2OLooAuto {

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
            // Backdrop side will be at higher motor speed so it is quicker to complete tasks to avoid collision.
            case LEFT:
//                1. Perform a smooth/swing 55° CCW/left turn as the robot strafe left for 30" at 80% speed.
                driveTrain.EncoderAutoMecanumDrive(0, -30, -55, 0.8, 2);
//                2. Strafe right 7" to release/leave the purple pixel at the left spike mark.
                driveTrain.EncoderAutoMecanumDrive(0, 7, 0, 0.8, 1);
//                3. Drive forward 10"towards to line up robot roughly in the middle between the wingnuts holding the backdrop when it rotates.
                driveTrain.EncoderAutoMecanumDrive(10, 0, 0, 0.8, 1);
//                4. Rotate 118° CCW so the rear of the robot faces the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 0, -118, 0.5, 3);
//                5. Reverse 38" to hit and square the robot against the backdrop.
                driveTrain.EncoderAutoMecanumDrive(-32, 0, 0, 0.5, 3);
//                6. Drive forward 18" to provide space for the scoring arm.
                driveTrain.EncoderAutoMecanumDrive(18,0,0,0.5,2);
//                7. Strafe right 6" to align to LEFT AprilTag
                driveTrain.EncoderAutoMecanumDrive(0,6,0,0.5,1);
//                ## The robot is in position to score.

                break;
            case CENTER:
//                1. Strafe left 32" to deliver the pixel.
                driveTrain.EncoderAutoMecanumDrive(0, -32, 0, 0.8, 2);
//                2. Strafe right 3" to leave the pixel at the spike mark, and align to CENTER AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0, 3, 0, 0.8, 1);
//                3. Drive forward 30" so the robot will have room to rotate (without hitting the pixel).
                driveTrain.EncoderAutoMecanumDrive(30, 0, 0, 0.8, 2);
//                4. Rotate 175° CW so the rear of the robot faces the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 0, 175, 0.5, 3);
//                5. Reverse 8" to hit and square the robot against the backdrop.
                driveTrain.EncoderAutoMecanumDrive(-8, 0, 0, 0.5, 1);
//                6. Drive forward 18" to provide space for the scoring arm.
                driveTrain.EncoderAutoMecanumDrive(18, 0, 0, 0.5, 2);
//                ## The robot is in position to score.
                break;
            case RIGHT:
//                1. Perform a diagonal strafe (10" reverse and 26" left) to deliver the pixel.
                driveTrain.EncoderAutoMecanumDrive(10, -26, 0, 0.8, 3);
//                2. Strafe right 5" to align to RIGHT AprilTag.
                driveTrain.EncoderAutoMecanumDrive(0, 5, 0, 0.8, 1);
//                3. Rotate 175° CW so the rear of the robot faces the backdrop.
                driveTrain.EncoderAutoMecanumDrive(0, 0, 175, 0.5, 3);
//                4. Reverse 10" to get to the scoring zone prep position.
                driveTrain.EncoderAutoMecanumDrive(-10, 0, 0, 0.8, 1);
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
        // Still need to move the robot backwards to land the claw onto the backdrop (slowly).
        driveTrain.EncoderAutoMecanumDrive(-5, 0, 0, 0.2, 3);
        // Still need to release the claw to drop the yellow pixel. Need to create separate function to open claw or add into scoreAuto.
        attachmentControl.dropYellowAuto();
        sleep(250);
        attachmentControl.resetArmAuto();
//        sleep(250);

        /*
        Parking path case statements.
        Objective is to park in F6 zone.
         */
        switch (location) {
            case LEFT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -39, 0, 0.8, 3);//20
                driveTrain.EncoderAutoMecanumDrive(-18, 0, 0, 0.8, 3);
                break;
            case CENTER:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -31, 0, 0.8, 3);
                driveTrain.EncoderAutoMecanumDrive(-18, 0, 0, 0.8, 3);
                break;
            case RIGHT:
                /*
                1. Strafe right x" (at 80% speed) to clear the backdrop.
                2. Reverse straight to park in tile D6 (see Game Manual 2 Appendix B)
                 */
                driveTrain.EncoderAutoMecanumDrive(0, -22, 0, 0.8, 3);//36
                driveTrain.EncoderAutoMecanumDrive(-18, 0, 0, 0.8, 3);
                break;
            case NONE:
                break;

        }

        while (attachmentControl.extendArmMotor.isBusy() || attachmentControl.rotateArmMotor.isBusy()); // Keep robot operating until all motor functions are completed.

    }

}
