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

    DcMotor intakeMotorLeft, intakeMotorRight;
    MotorControlGroup intakeGroup;
    Servo droneServo;
    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl, Gamepad gamepad1, Gamepad gamepad2) {

        this.telemetryControl = telemetryControl;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        droneServo = hardwareMap.servo.get("droneServo");
        droneServo.scaleRange(0.35, 1);

        intakeMotorLeft = hardwareMap.dcMotor.get("intakeMotorLeft");
        intakeMotorRight = hardwareMap.dcMotor.get("intakeMotorRight");
        intakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeGroup = new MotorControlGroup(intakeMotorLeft, intakeMotorRight);

        

    }

    public void droneManual(boolean increaseButton, boolean decreaseButton) {
        double position = droneServo.getPosition();
        if (increaseButton) position += 0.005;
        else if (decreaseButton) position -= 0.05;

        if (position > 1) position = 1;
        if (position < 0) position = 0;
        droneServo.setPosition(position);

        telemetryControl.addData("Drone Servo Position", droneServo.getPosition());

    }

    public void droneTeleOp(boolean button) {

        if (button) {

            if (droneServo.getPosition() > 0.85) {

                droneServo.setPosition(0);

            } else {

                droneServo.setPosition(1);

            }

        }

    }

    public void intakeManual(double power) {

        intakeGroup.setPower(power);

    }

    public void intakeTeleOp() {
        if (gamepad1.right_bumper) {

            if (!lastRightBumper && intakeGroup.getPower() < 0.35) {

                intakeGroup.setPower(0.375);

            } else if (!lastRightBumper) {

                intakeGroup.setPower(0);

            }

            lastRightBumper = true;

        } else {

            lastRightBumper = false;

        }

        if (gamepad1.left_bumper) {

            if (!lastLeftBumper && intakeGroup.getPower() > -0.85) {

                intakeGroup.setPower(-1);

            } else if (!lastLeftBumper) {

                intakeGroup.setPower(0);

            }

            lastLeftBumper = true;

        } else {

            lastLeftBumper = false;

        }
    }

    public void intakeAuto() {

        intakeGroup.setPower(-1);

    }

}
