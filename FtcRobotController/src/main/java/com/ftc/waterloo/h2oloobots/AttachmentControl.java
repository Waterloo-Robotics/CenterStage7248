package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    DcMotor intakeMotorLeft, intakeMotorRight, extendArmMotor, rotateArmMotor;
    MotorControlGroup intakeGroup;
    Servo droneServo;
    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;

    DcMotor hangMotor;
    Servo hangServo;
    boolean isGP2APressed = false;

    boolean isAPressed = false;

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, Gamepad gamepad1, Gamepad gamepad2) {

        this.telemetryControl = telemetryControl;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        droneServo = hardwareMap.servo.get("droneServo");
        droneServo.scaleRange(0.35, 1);
        droneServo.setPosition(1);

        intakeMotorLeft = hardwareMap.dcMotor.get("intakeMotorLeft");
        intakeMotorRight = hardwareMap.dcMotor.get("intakeMotorRight");
        intakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeGroup = new MotorControlGroup(intakeMotorLeft, intakeMotorRight);

        hangMotor = hardwareMap.dcMotor.get("hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangServo = hardwareMap.servo.get("hangServo");
        hangServo.scaleRange(0.49, 0.94);

        extendArmMotor = hardwareMap.dcMotor.get("extendArmMotor");
        rotateArmMotor = hardwareMap.dcMotor.get("rotateArmMotor");

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

    public void intakeManual(double power) {

        intakeGroup.setPower(power);

    }

    public void intakeTeleOp() {
        if (gamepad1.right_bumper) {

            if (!lastRightBumper && intakeGroup.getPower() < 0.5) {

                intakeGroup.setPower(0.6);

            } else if (!lastRightBumper) {

                intakeGroup.setPower(0);

            }

            lastRightBumper = true;

        } else {

            lastRightBumper = false;

        }

        if (gamepad1.left_bumper) {

            if (!lastLeftBumper && intakeGroup.getPower() > -0.10) {

                intakeGroup.setPower(-0.15);

            } else if (!lastLeftBumper) {

                intakeGroup.setPower(0);

            }

            lastLeftBumper = true;

        } else {

            lastLeftBumper = false;

        }
    }

    public void intakeAuto() {

        intakeGroup.setPower(-20);

    }

    public void intakeAutoWithPower(double intakeMotorPower) {

        intakeGroup.setPower(intakeMotorPower);

    }
    public void extendArmMotorManual() {

        extendArmMotor.setPower(gamepad2.right_stick_x);

    }

    public void rotateArmMotorManual() {

        rotateArmMotor.setPower(gamepad2.right_stick_y);

    }

}
