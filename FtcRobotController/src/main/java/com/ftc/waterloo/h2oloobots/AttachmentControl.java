package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public DcMotorEx extendArmMotor;
    public DcMotorEx rotateArmMotor;
    ElapsedTime extendTime = new ElapsedTime();
    boolean isExtendTimeStarted = true;
    Servo clawRotate;
    Servo clawPickupLeft, clawPickupRight;
    Servo droneServo;
    boolean firstStepCompleted = true;
    boolean lastAButton = false, lastBButton = false, lastXButton = false;
    boolean isStartOfHang0 = true;
    ElapsedTime hangTime = new ElapsedTime();
    public enum ArmPosition {
        LOW,
        MED,
        HIGH,
        PICKUP,
        CARRY,
        HANG_UP,
        HANG_LATCH
    }

    public int hangStep = 0;

    ArmPosition armPosition = ArmPosition.CARRY;

    DcMotor hangMotor;
    Servo hangServo;
    boolean isGP2APressed = false;

    boolean isAPressed = false;

    TouchSensor bottomTouch, topTouch;
    TouchSensor extendTouch;

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, Gamepad gamepad1, Gamepad gamepad2) {

        this.telemetryControl = telemetryControl;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        droneServo = hardwareMap.servo.get("droneServo");
        droneServo.scaleRange(0.86, 0.9);
        droneServo.setPosition(0);

        hangMotor = hardwareMap.dcMotor.get("hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangServo = hardwareMap.servo.get("hangServo");
        hangServo.scaleRange(0, 0.35);
        hangServo.setPosition(1);

        clawRotate = hardwareMap.servo.get("clawRotate");
        clawRotate.scaleRange(0.569, 0.968);
        clawRotate.setPosition(1);
        clawPickupLeft = hardwareMap.servo.get("clawPickupLeft");
        clawPickupLeft.scaleRange(0.45, 0.63);
        clawPickupLeft.setPosition(0);
        clawPickupRight = hardwareMap.servo.get("clawPickupRight");
        clawPickupRight.scaleRange(0.64, 0.86);
        clawPickupRight.setPosition(1);

        bottomTouch = hardwareMap.touchSensor.get("bottomTouch");
        topTouch = hardwareMap.touchSensor.get("topTouch");
        extendTouch = hardwareMap.touchSensor.get("extendTouch");
        /*
        Map the hardware configuration to software. Best practice is to use the same name between HW configuration and SW.
        * Best practice is to default zero power behavior to BRAKE (instead of FLOAT)
        * Best practice to reset the position encoder on the port (set to zero)
        * RUN_WITHOUT_ENCODER does not disable the encoder. It instead tells the SDK not to use the motor encoder for built-in velocity control
        */
        extendArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("extendArmMotor");
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (!extendTouch.isPressed()) extendArmMotor.setPower(1); //When class is initialize, arm extension retract to touch sensor to set position 0.
        extendArmMotor.setPower(0); //kill power to motor because arm has fully retracted.
        extendArmMotor.setTargetPosition(0);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //the current encoder reading is set to position 0.
        extendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  //run motor without internal PID for velocity control.
        extendArmMotor.setTargetPositionTolerance(30); //set current encoder tolerance to 30 encoder counts... don't know limit or best practice.

        rotateArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("rotateArmMotor");
        rotateArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (!bottomTouch.isPressed()) rotateArmMotor.setPower(-1); //When class is initialize, rotate arm until depress bottom touch sensor to set position 0.
        rotateArmMotor.setPower(0);
        rotateArmMotor.setTargetPosition(0);
        rotateArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotateArmMotor.setTargetPositionTolerance(70);

    }

    public void setRotateArmMotor(double power) {

        if ((bottomTouch.isPressed() && power < 0) || (topTouch.isPressed() && power > 0)) {

            rotateArmMotor.setPower(0);

        } else {

            rotateArmMotor.setPower(power);

        }

    }

    public void setRotateArmMotorEncoder() {

        if ((bottomTouch.isPressed() && (rotateArmMotor.getCurrentPosition() > rotateArmMotor.getTargetPosition())) || (topTouch.isPressed() && (rotateArmMotor.getCurrentPosition() < rotateArmMotor.getTargetPosition()))) {

            rotateArmMotor.setPower(0);

        } else {

            rotateArmMotor.setPower(1);

        }

    }

    public void setExtendArmMotorEncoder() {

        if ((extendTouch.isPressed() && extendArmMotor.getCurrentPosition() > -100 && extendArmMotor.getTargetPosition() > -75)) {

            rotateArmMotor.setPower(0);

        } else {

            rotateArmMotor.setPower(1);

        }

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

    public void armTeleOp() {

        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmMotor.setTargetPositionTolerance(70);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateArmMotor.setTargetPositionTolerance(70);

        if (gamepad2.dpad_up) {

            armPosition = ArmPosition.HIGH;
            clawRotate.setPosition(1);
            firstStepCompleted = rotateArmMotor.getCurrentPosition() > 450;

        } else if (gamepad2.dpad_left || gamepad2.dpad_right) {

            armPosition = ArmPosition.MED;
            clawRotate.setPosition(1);
            firstStepCompleted = rotateArmMotor.getCurrentPosition() > 450;

        } else if (gamepad2.dpad_down) {

            armPosition = ArmPosition.LOW;
            clawRotate.setPosition(1);
            firstStepCompleted = rotateArmMotor.getCurrentPosition() > 450;

        } else if (gamepad2.b) {

            armPosition = ArmPosition.CARRY;
            clawRotate.setPosition(1);
            firstStepCompleted = rotateArmMotor.getCurrentPosition() < 550;
            clawPickupRight.setPosition(1);
            clawPickupLeft.setPosition(0);

        } else if (gamepad2.a) {

            armPosition = ArmPosition.PICKUP;
            clawRotate.setPosition(0);
            firstStepCompleted = rotateArmMotor.getCurrentPosition() < 550;
            clawPickupRight.setPosition(0);
            clawPickupLeft.setPosition(1);

        } else if (gamepad2.left_bumper) {

            armPosition = ArmPosition.HANG_UP;
            firstStepCompleted = rotateArmMotor.getCurrentPosition() > 450;
            clawRotate.setPosition(1);

        } else if (gamepad2.right_bumper) {

            armPosition = ArmPosition.HANG_LATCH;
            isStartOfHang0 = true;
            firstStepCompleted = true;
            clawRotate.setPosition(1);

        }

        switch (armPosition) {

            case HIGH:

                if (firstStepCompleted) {
                    rotateArmMotor.setTargetPosition(4391);
                    extendArmMotor.setTargetPosition(-2683);
                }
                break;

            case MED:

                if (firstStepCompleted) {
                    rotateArmMotor.setTargetPosition(4731);
                    extendArmMotor.setTargetPosition(-2137);
                }
                break;

            case LOW:

                if (firstStepCompleted) {
                    rotateArmMotor.setTargetPosition(4900);
                    extendArmMotor.setTargetPosition(-1225);
                }
                break;

            case PICKUP:

                if (firstStepCompleted) {
                    rotateArmMotor.setTargetPosition(1);
                    extendArmMotor.setTargetPosition(-403);
                }
                break;

            case CARRY:

                if (firstStepCompleted) {
                    rotateArmMotor.setTargetPosition(0);
                    extendArmMotor.setTargetPosition(0);
                }
                break;

            case HANG_UP:
                if (firstStepCompleted) {
                    rotateArmMotor.setTargetPosition(3163);
                    extendArmMotor.setTargetPosition(-898);
                }
                break;

            case HANG_LATCH:
                if (isStartOfHang0) {
                    extendArmMotor.setTargetPosition(-590);
                    rotateArmMotor.setTargetPosition(3163);
                    isStartOfHang0 = false;
                }

                if ((extendArmMotor.getCurrentPosition() < -580 && extendArmMotor.getCurrentPosition() > -600) && !isStartOfHang0) {
                    rotateArmMotor.setTargetPosition(500);
                    clawRotate.setPosition(0);
                }
                break;
        }

        if (!firstStepCompleted) {

            extendArmMotor.setTargetPosition(0);
            rotateArmMotor.setTargetPosition(800);

            if ((rotateArmMotor.getCurrentPosition() < 900 && rotateArmMotor.getCurrentPosition() > 800) && (extendArmMotor.getCurrentPosition() < 100 && extendArmMotor.getCurrentPosition() > -100)) {

                firstStepCompleted = true;

            }

        }

        this.setRotateArmMotorEncoder();
        this.setExtendArmMotorEncoder();

        telemetryControl.addData("Extend Arm Current Position", extendArmMotor.getCurrentPosition());
        telemetryControl.addData("Extend Arm Target Position", extendArmMotor.getTargetPosition());
        telemetryControl.addData("Extend Arm Error", extendArmMotor.getCurrentPosition() - extendArmMotor.getTargetPosition());
        telemetryControl.addData("Extend Arm Is Busy", extendArmMotor.isBusy());
        telemetryControl.addData("Rotate Arm Current Position", rotateArmMotor.getCurrentPosition());
        telemetryControl.addData("Rotate Arm Target Position", rotateArmMotor.getTargetPosition());
        telemetryControl.addData("Rotate Arm Is Busy", rotateArmMotor.isBusy());
        telemetryControl.addData("Desired Position", armPosition);
        telemetryControl.addData("Is First Step Completed", firstStepCompleted);

    }

    public void intermediateAuto() {

        extendArmMotor.setTargetPosition(0);
        rotateArmMotor.setTargetPosition(500);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendArmMotor.setPower(1);
        rotateArmMotor.setPower(1);

    }

    public void scoreAuto() {

        rotateArmMotor.setTargetPosition(5100);//low line is at 4900. We want to go lower.
        extendArmMotor.setTargetPosition(-1225);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendArmMotor.setPower(1);
        rotateArmMotor.setPower(1);

    }

    public void resetArmAuto() {

        clawPickupRight.setPosition(1);
        clawPickupLeft.setPosition(0);
        rotateArmMotor.setTargetPosition(0);
        extendArmMotor.setTargetPosition(0);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendArmMotor.setPower(1);
        rotateArmMotor.setPower(1);

    }
    public void dropYellowAuto() {

        clawPickupRight.setPosition(0);
        clawPickupLeft.setPosition(1);

    }

    public void clawPickupTeleOp(boolean leftButton, boolean rightButton, boolean bothButton) {

        if (!armPosition.equals(ArmPosition.CARRY)) {

            if (bothButton) {

                if (!lastAButton) {

                    if (clawPickupRight.getPosition() < 0.1 || clawPickupLeft.getPosition() > 0.9) {

                        clawPickupRight.setPosition(1);
                        clawPickupLeft.setPosition(0);

                    } else {

                        clawPickupRight.setPosition(0);
                        clawPickupLeft.setPosition(1);

                    }

                }

                lastAButton = true;

            } else {

                lastAButton = false;

            }

            if (rightButton) {

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

            if (leftButton) {

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

            hangServo.setPosition(hangServo.getPosition() + 0.005);

        } else if (gamepad2.b) {

            hangServo.setPosition(hangServo.getPosition() - 0.005);

        }

        telemetryControl.addData("Hang Servo Position", hangServo.getPosition());


    }

    public void hangServoTeleOp() {

        if (gamepad2.left_trigger > 0.1) {

            if (!isGP2APressed) {

                if (hangServo.getPosition() < 0.05) {

                    hangServo.setPosition(1);

                } else {

                    hangServo.setPosition(0);

                }


            }

            isGP2APressed = true;

        } else {

            isGP2APressed = false;

        }

        telemetryControl.addData("Hang Servo Position", hangServo.getPosition());

    }

    public void droneTeleOp() {

        if (gamepad1.right_trigger>0.1) {

            if (!isAPressed) {

                if (droneServo.getPosition() > 0.85) {

                    droneServo.setPosition(0);

                } else {

                    droneServo.setPosition(0.88);

                }

            }

            isAPressed = true;

        } else {

            isAPressed = false;

        }

    }

    public void extendArmMotorManual() {

        if ((extendArmMotor.getCurrentPosition() > -10 && gamepad2.left_stick_y > 0) || (extendArmMotor.getCurrentPosition() < -2690 && gamepad2.left_stick_y < 0)) {

            extendArmMotor.setPower(0);

        } else {

            extendArmMotor.setPower(gamepad2.left_stick_y);

        }

        if (gamepad2.dpad_up) clawRotate.setPosition(1);
        else if (gamepad2.dpad_down) clawRotate.setPosition(0);
        telemetryControl.addData("Extend Arm Position", extendArmMotor.getCurrentPosition());

    }

    public void rotateArmMotorManual() {

        setRotateArmMotor(gamepad2.right_stick_y);
        telemetryControl.addData("Rotate Arm Position", rotateArmMotor.getCurrentPosition());

    }

    public void touchSensorTelemetry() {

        telemetryControl.addData("Bottom Touch", bottomTouch.isPressed());
        telemetryControl.addData("Top Touch", topTouch.isPressed());
        telemetryControl.addData("Extend Touch", extendTouch.isPressed());

    }

}
