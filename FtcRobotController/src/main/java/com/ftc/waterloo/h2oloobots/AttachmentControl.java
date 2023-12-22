package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**This AttachmentControl class just offers a global area to store any non-drivebase commands and
 * devices to be used as chosen. Most imports should be added already, but if you need other
 * imports Android Studio is pretty good at auto importing.*/
public class AttachmentControl {

    /** This declares a global telemetryControl variable to make our lives easier, we set these when
     * we initialise the file. You can use this telemetryControl variable to add telemetry without
     * having to route it from the opMode to this class.
    */
    TelemetryControl telemetryControl;
    Gamepad gamepad1, gamepad2;

    DcMotor extendArmMotor, rotateArmMotor;
    Servo clawRotate;
    Servo clawPickupLeft, clawPickupRight;
    Servo droneServo;
    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;
    boolean lastAButton = false, lastBButton = false, lastXButton = false;

    DcMotor hangMotor;
    Servo hangServo;
    boolean isGP2APressed = false;

    boolean isAPressed = false;

    TouchSensor bottomTouch, topTouch;

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, Gamepad gamepad1, Gamepad gamepad2) {

        this.telemetryControl = telemetryControl;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        droneServo = hardwareMap.servo.get("droneServo");
        droneServo.scaleRange(0.35, 1);
        droneServo.setPosition(1);

        hangMotor = hardwareMap.dcMotor.get("hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangServo = hardwareMap.servo.get("hangServo");
        hangServo.scaleRange(0.49, 0.94);

        clawRotate = hardwareMap.servo.get("clawRotate");
        clawRotate.scaleRange(0.569, 0.968);
        clawPickupLeft = hardwareMap.servo.get("clawPickupLeft");
        clawPickupLeft.scaleRange(0.45, 0.63);
        clawPickupRight = hardwareMap.servo.get("clawPickupRight");
        clawPickupRight.scaleRange(0.64, 0.86);
        /*
        Map the hardware configuration to software. Best practice is to use the same name between HW configuration and SW.
        * Best practice is to default zero power behavior to BRAKE (instead of FLOAT)
        * Best practice to reset the position encoder on the port (set to zero)
        * RUN_WITHOUT_ENCODER does not disable the encoder. It instead tells the SDK not to use the motor encoder for built-in velocity control
        */
        extendArmMotor = hardwareMap.dcMotor.get("extendArmMotor");
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotateArmMotor = hardwareMap.dcMotor.get("rotateArmMotor");
        rotateArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bottomTouch = hardwareMap.touchSensor.get("bottomTouch");
        topTouch = hardwareMap.touchSensor.get("topTouch");

    }

    public void clawPickupManual() {

        if (gamepad2.dpad_up) {

            clawPickupLeft.setPosition(clawPickupLeft.getPosition() + 0.03);

        } else if (gamepad2.dpad_down) {

            clawPickupLeft.setPosition(clawPickupLeft.getPosition() - 0.03);

        }

        if (gamepad2.y) {

            clawPickupRight.setPosition(clawPickupRight.getPosition() + 0.03);

        } else if (gamepad2.a) {

            clawPickupRight.setPosition(clawPickupRight.getPosition() - 0.03);

        }

        telemetryControl.addData("Claw Pickup Left Position", clawPickupLeft.getPosition());
        telemetryControl.addData("Claw Pickup Right Position", clawPickupRight.getPosition());

    }

    public void clawPickupTeleOp() {

        if (gamepad2.a) {

            if (!lastAButton) {

                if (clawPickupRight.getPosition() == 0) {

                    clawPickupRight.setPosition(1);

                } else {

                    clawPickupRight.setPosition(0);

                }

                if (clawPickupLeft.getPosition() == 0) {

                    clawPickupLeft.setPosition(1);

                } else {

                    clawPickupLeft.setPosition(0);

                }

            }

            lastAButton = true;

        } else {

            lastAButton = false;

        }

        if (gamepad2.b) {

            if (!lastBButton) {

                if (clawPickupRight.getPosition() == 0) {

                    clawPickupRight.setPosition(1);

                } else {

                    clawPickupRight.setPosition(0);

                }

            }

            lastBButton = true;

        } else {

            lastBButton = false;

        }

        if (gamepad2.x) {

            if (!lastXButton) {

                if (clawPickupLeft.getPosition() == 0) {

                    clawPickupLeft.setPosition(1);

                } else {

                    clawPickupLeft.setPosition(0);

                }

            }

            lastXButton = true;

        } else {

            lastXButton = false;

        }

    }

    public void clawRotateManual() {

        if (gamepad2.left_bumper) {

            clawRotate.setPosition(clawRotate.getPosition() + 0.03);

        } else if (gamepad2.right_bumper) {

            clawRotate.setPosition(clawRotate.getPosition() - 0.03);

        }

        telemetryControl.addData("Claw Rotate Position", clawRotate.getPosition());

    }

    public void hangMotorManual() {

        hangMotor.setPower(gamepad2.left_stick_y);

    }

    public void hangServoManual() {

        if (gamepad2.a) {

            hangServo.setPosition(hangServo.getPosition() + 0.001);

        } else if (gamepad2.b) {

            hangServo.setPosition(hangServo.getPosition() - 0.001);

        }

        telemetryControl.addData("Hang Servo Position", hangServo.getPosition());


    }

    public void hangServoTeleOp() {

        if (gamepad2.a) {

            if (!isGP2APressed) {

                if (hangServo.getPosition() < 0.05) {

                    hangServo.setPosition(0.832);

                } else if (hangServo.getPosition() < 0.85) {

                    hangServo.setPosition(1);

                } else {

                    hangServo.setPosition(0);

                }


            }

            isGP2APressed = true;

        } else {

            isGP2APressed = false;

        }

    }

    public void droneManual() {
        double position = droneServo.getPosition();
        if (gamepad2.a) position += 0.005;
        else if (gamepad2.b) position -= 0.05;

        if (position > 1) position = 1;
        if (position < 0) position = 0;
        droneServo.setPosition(position);

        telemetryControl.addData("Drone Servo Position", droneServo.getPosition());

    }

    public void droneTeleOp() {

        if (gamepad1.a) {

            if (!isAPressed) {

                if (droneServo.getPosition() > 0.85) {

                    droneServo.setPosition(0);

                } else {

                    droneServo.setPosition(1);

                }

            }

            isAPressed = true;

        } else {

            isAPressed = false;

        }

    }

    public void extendArmMotorManual() {

        if ((extendArmMotor.getCurrentPosition() > -10 && gamepad2.left_stick_y > 0) || (rotateArmMotor.getCurrentPosition() > -200 && extendArmMotor.getCurrentPosition() < -1200 && gamepad2.left_stick_y < 0) || (rotateArmMotor.getCurrentPosition() <= -200 && extendArmMotor.getCurrentPosition() < -2690 && gamepad2.left_stick_y < 0)) {

            extendArmMotor.setPower(0);

        } else {

            extendArmMotor.setPower(gamepad2.left_stick_y);

        }
        telemetryControl.addData("Extend Arm Position", extendArmMotor.getCurrentPosition());

    }

    public void rotateArmMotorManual() {

        if ((bottomTouch.isPressed() && gamepad2.right_stick_y < 0) || (topTouch.isPressed() && gamepad2.right_stick_y > 0)) {

            rotateArmMotor.setPower(0);

        } else if (rotateArmMotor.getCurrentPosition() > -500){

            rotateArmMotor.setPower(gamepad2.right_stick_y * 0.33);

        } else {

            rotateArmMotor.setPower(gamepad2.right_stick_y * 0.75);

        }
        telemetryControl.addData("Rotate Arm Position", rotateArmMotor.getCurrentPosition());

    }

    public void touchSensorTelemetry() {

        telemetryControl.addData("Bottom Touch", bottomTouch.isPressed());
        telemetryControl.addData("Top Touch", topTouch.isPressed());

    }

}
