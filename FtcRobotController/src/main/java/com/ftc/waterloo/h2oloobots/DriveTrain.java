package com.ftc.waterloo.h2oloobots;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {


    public enum DriveTrainType {

        TWO_WHEEL_DRIVE,
        FOUR_WHEEL_TANK,
        MECANUM

    }

    public enum DriveDirection {

        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TURN_CLOCKWISE,
        TURN_COUNTERCLOCKWISE,
        DIAGONAL_FORWARD_RIGHT,
        DIAGONAL_FORWARD_LEFT,
        DIAGONAL_BACKWARD_RIGHT,
        DIAGONAL_BACKWARD_LEFT,

    }

    DriveTrainType driveTrainType = DriveTrainType.MECANUM;

    // defines drive motors for a 4 wheel drive
    DcMotor fl, fr, bl, br;

    // defines drive motors for a two wheel drive
    DcMotor left, right;

    /* counts per revolution of the drive motors. None of this is necessary if you aren't using
     * the encoder drive in this file. */
    double countsPerRevolution = 553;

    // diameter of your drive wheels
    double wheelDiameter = 96 / 25.4; // inches
    double wheelCircumference = wheelDiameter * Math.PI;

    /*
     * Track Width is the distance between the two sets of wheels (defined by the line of x below).
     *
     * TODO this will need to be updated for turns to work. it might not be the actual value
     *  that is physically measured.
     *
     *          FRONT
     *  O--------------------O
     *  |                    |
     *  |                    |
     *  |                    |
     *  |xxxxxxxxxxxxxxxxxxxx|
     *  |                    |
     *  |                    |
     *  |                    |
     *  O--------------------O
     */
    double trackWidth = 27.25;
    // how many inches the wheels travel in a full rotation of the robot
    double fullRotation = trackWidth * Math.PI;

    // calculates how many encoder counts are in an inch and in a degree
    double COUNTS_PER_INCH = countsPerRevolution / wheelCircumference;
    double COUNTS_PER_DEGREE = fullRotation / wheelCircumference * countsPerRevolution / 360.0;

    // defines local HardwareMap and TelemetryControl variables.
    HardwareMap hardwareMap;
    TelemetryControl telemetryControl;
    AttachmentControl attachmentControl;

    /**Initialises the drivetrain variable.
     * @param hardwareMap the local HardwareMap variable from in the runOpMode() void.
     * @param telemetryControl the TelemetryControl variable initialized in the runOpMode() void.
     * */
    public DriveTrain(HardwareMap hardwareMap,
                      TelemetryControl telemetryControl
    ) {

        this.hardwareMap = hardwareMap;
        this.telemetryControl = telemetryControl;
        this.FourMotorInit();

    }

    /**Initialises the drivetrain variable.
     * @param hardwareMap the local HardwareMap variable from in the runOpMode() void.
     * @param telemetryControl the TelemetryControl variable initialized in the runOpMode() void.
     * @param zeroPowerBehavior the zero power behavior to be set to the drive motors.*/
    public DriveTrain(
            HardwareMap hardwareMap,
            TelemetryControl telemetryControl,
            AttachmentControl attachmentControl,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {

        this.hardwareMap = hardwareMap;
        this.telemetryControl = telemetryControl;
        this.attachmentControl = attachmentControl;
        this.FourMotorInit(zeroPowerBehavior);

    }

    /**Sets the drivetrain type after the constructor.
     * @param driveTrainType the DriveTrain type to be set*/
    public void setDriveTrainType(@NonNull DriveTrainType driveTrainType) {

        this.driveTrainType = driveTrainType;

        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
                this.TwoWheelInit();
                break;

            case FOUR_WHEEL_TANK:
            case MECANUM:
                this.FourMotorInit();
                break;

            default:
                this.FourMotorInit();
                break;

        }

    }

    /**Sets the drivetrain type after the constructor.
     * @param driveTrainType the DriveTrain type to be set
     * @param zeroPowerBehavior the behavior of the motors when they are not receiving power*/
    public void setDriveTrainType(@NonNull DriveTrainType driveTrainType, @NonNull DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        this.driveTrainType = driveTrainType;

        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
                this.TwoWheelInit(zeroPowerBehavior);
                break;

            case FOUR_WHEEL_TANK:
            case MECANUM:
                this.FourMotorInit(zeroPowerBehavior);
                break;

            default:
                this.FourMotorInit(zeroPowerBehavior);
                break;

        }

    }

    /**Initialization for two motor drivebase (one left, one right).*/
    void TwoWheelInit() {

        this.TwoWheelInit(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**Initialization for two motor drivebase (one left, one right).
     * @param zeroPowerBehavior the zero power behavior to set to the drive motors.*/
    void TwoWheelInit(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(zeroPowerBehavior);
        right.setZeroPowerBehavior(zeroPowerBehavior);

    }

    /**Four motor initialization command for any four motor drive base with two motors on either
     * side. Could be mecanum or tank.*/
    void FourMotorInit() {

        this.FourMotorInit(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**Four motor initialization command for any four motor drive base with two motors on either
     * side. Could be mecanum or tank.
     * @param zeroPowerBehavior the zero power behavior to set to the motors.*/
    void FourMotorInit(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**TeleOp code for a 4 wheel tank or two wheel drive.
     * @param FBInput forward and back input, range -1 to 1
     * @param pivotInput pivot input, range -1 to 1*/
    public void teleOpDrive(double FBInput, double pivotInput) {

        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
                this.TwoWheelDriveTeleOp(FBInput, pivotInput);
                telemetryControl.motorTelemetryUpdate(left.getPower(), right.getPower());
                break;

            case FOUR_WHEEL_TANK:
                this.FWDTeleOp(FBInput, pivotInput);
                telemetryControl.motorTelemetryUpdate(fl.getPower(), fr.getPower(), bl.getPower(), br.getPower());
                break;

            case MECANUM:
                RobotLog.setGlobalErrorMsg("Passing in a LRInput is necessary. Please use the " +
                        "version of this function that uses 'teleOpDrive(FBInput, LRInput," +
                        " pivotInput)'.");
                break;

        }

    }

    /**TeleOp code for a mecanum drive.
     * @param FBInput forward and back input, range -1 to 1
     * @param LRInput left and right strafing input, range -1 to 1
     * @param pivotInput pivot input, range -1 to 1*/
    public void teleOpDrive(double FBInput, double LRInput, double pivotInput, AttachmentControl attachmentControl) {

        double speedMul;
        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
            case FOUR_WHEEL_TANK:
                RobotLog.setGlobalErrorMsg("Passing in a LRInput is unacceptable. Please use the " +
                        "version of this function that uses 'teleOpDrive(FBInput," +
                        " pivotInput)'.");
                break;

            case MECANUM:

                if (attachmentControl.hangServo.getPosition() > 0.8) {

                    speedMul = 0.25;

                } else {

                    speedMul = 0.75;

                }
                this.MecanumTeleOp(FBInput * speedMul, LRInput * speedMul, pivotInput * speedMul);
                break;

        }

    }

    /**Simple two wheel drive TeleOp.
     * @param FBInput input used for forward and back movements.
     * @param PivotInput input used for turning.*/
    public void TwoWheelDriveTeleOp(double FBInput, double PivotInput) {

        right.setPower(-FBInput - PivotInput);
        left.setPower(-FBInput + PivotInput);

    }

    /**Simple four wheel tank drive TeleOp.
     * @param FBInput input used for forward and back movements.
     * @param PivotInput input used for turning.*/
    public void FWDTeleOp(double FBInput, double PivotInput) {

        fr.setPower(-FBInput - PivotInput);
        fl.setPower(-FBInput + PivotInput);
        br.setPower(-FBInput - PivotInput);
        bl.setPower(-FBInput + PivotInput);

    }

    /**Simple Mecanum drive TeleOp.
     * @param FBInput input used for forward and back movements.
     * @param LRInput input used for strafing left and right.
     * @param PivotInput input used for turning.*/
    public void MecanumTeleOp(double FBInput, double LRInput, double PivotInput) {


        double frPower = -FBInput - LRInput - (PivotInput * 0.75);
        double brPower = -FBInput + LRInput - (PivotInput * 0.75);
        double flPower = -FBInput + LRInput + (PivotInput * 0.75);
        double blPower = -FBInput - LRInput + (PivotInput * 0.75);

        fr.setPower(frPower);
        br.setPower(brPower);
        fl.setPower(flPower);
        bl.setPower(blPower);

//        telemetryControl.motorTelemetryUpdate(
//                fl.getPower(),
//                fr.getPower(),
//                bl.getPower(),
//                br.getPower()
//        );

    }

    /**Simple Autonomous code to drive at set powers for a set time.
     * @param FRPower The power to be set to the front right motor.
     * @param FLPower The power to be set to the front left motor.
     * @param BRPower The power to be set to the back right motor.
     * @param BLPower The power to be set to the back left motor.
     * @param SECONDS The time for the movement to occur over.*/
    public void timeAutoDrive(double FRPower, double FLPower, double BRPower, double BLPower, double SECONDS) {

        if (this.driveTrainType == DriveTrainType.TWO_WHEEL_DRIVE) {

            RobotLog.setGlobalErrorMsg("You must use only two power arguments for a 2 wheel drive.");

        } else {

            this.timeAutoFourWheelDrive(FRPower, FLPower, BRPower, BLPower, SECONDS);

        }

    }

    /**Simple Autonomous code to drive at set powers for a set time.
     * @param leftPower The power to be set to the left motor.
     * @param rightPower The power to be set to the right motor.
     * @param SECONDS The time for the movement to occur over.*/
    public void timeAutoDrive(double leftPower, double rightPower, double SECONDS) {

        if (this.driveTrainType != DriveTrainType.TWO_WHEEL_DRIVE) {

            RobotLog.setGlobalErrorMsg("You must use four power arguments for a 4 wheel drive.");

        } else {

            this.timeAutoTwoWheelDrive(rightPower, leftPower, SECONDS);

        }

    }

    /**Simple Autonomous code to drive in a set direction for a set time.
     * @param driveDirection The direction for the robot to move.
     * @param SECONDS The time for the movement to occur over.*/
    public void timeAutoDrive(DriveDirection driveDirection, double power, double SECONDS) {

        switch (this.driveTrainType) {

            case TWO_WHEEL_DRIVE:
                this.timeAutoTwoWheelDrive(driveDirection, power, SECONDS);
                break;

            case MECANUM:
            case FOUR_WHEEL_TANK:
                this.timeAutoFourWheelDrive(driveDirection, power, SECONDS);
                break;

        }

    }

    /**Simple Four Wheel Drive Autonomous code to drive at set powers for a set time.
     * @param FRPower The power to be set to the front right motor.
     * @param FLPower The power to be set to the front left motor.
     * @param BRPower The power to be set to the back right motor.
     * @param BLPower The power to be set to the back left motor.
     * @param SECONDS The time for the movement to occur over.*/
    public void timeAutoFourWheelDrive(double FRPower, double FLPower, double BRPower, double BLPower, double SECONDS) {

        ElapsedTime time = new ElapsedTime();

        time.reset();

        while (time.seconds() < SECONDS) {

            fr.setPower(FRPower);
            fl.setPower(FLPower);
            br.setPower(BRPower);
            bl.setPower(BLPower);

        }

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

    }

    /**Simple Four Wheel Drive Autonomous code to drive in a set direction for a set time.
     * @param driveDirection The direction for the robot to move.
     * @param SECONDS The time for the movement to occur over.*/
    public void timeAutoFourWheelDrive(DriveDirection driveDirection, double power, double SECONDS) {

        double frPower = power;
        double flPower = power;
        double brPower = power;
        double blPower = power;

        switch (driveDirection) {

            case FORWARD:
                frPower *= -1;
                flPower *= -1;
                brPower *= -1;
                blPower *= -1;
                break;

            case BACKWARD:
                frPower *= 1;
                flPower *= 1;
                brPower *= 1;
                blPower *= 1;
                break;

            case STRAFE_LEFT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    frPower *= -1;
                    flPower *= 1;
                    brPower *= 1;
                    blPower *= -1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case STRAFE_RIGHT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    frPower *= 1;
                    flPower *= -1;
                    brPower *= -1;
                    blPower *= 1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case TURN_CLOCKWISE:
                frPower *= -1;
                flPower *= 1;
                brPower *= -1;
                blPower *= 1;
                break;

            case TURN_COUNTERCLOCKWISE:
                frPower *= 1;
                flPower *= -1;
                brPower *= 1;
                blPower *= -1;
                break;

            case DIAGONAL_FORWARD_LEFT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    frPower *= 1;
                    flPower *= 0;
                    brPower *= 0;
                    blPower *= 1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case DIAGONAL_FORWARD_RIGHT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    frPower *= 0;
                    flPower *= 1;
                    brPower *= 1;
                    blPower *= 0;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case DIAGONAL_BACKWARD_LEFT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    frPower *= -1;
                    flPower *= 0;
                    brPower *= 0;
                    blPower *= -1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case DIAGONAL_BACKWARD_RIGHT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    frPower *= 0;
                    flPower *= -1;
                    brPower *= -1;
                    blPower *= 0;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

        }

        this.timeAutoFourWheelDrive(frPower, flPower, brPower, blPower, SECONDS);

    }

    /**Simple Four Wheel Drive Autonomous code to drive at set powers for a set time.
     * @param rightPower The power to be set to the front right motor.
     * @param leftPower The power to be set to the front left motor.
     * @param SECONDS The time for the movement to occur over.*/
    public void timeAutoTwoWheelDrive(double rightPower, double leftPower, double SECONDS) {

        ElapsedTime time = new ElapsedTime();

        time.reset();

        while (time.seconds() < SECONDS) {

            right.setPower(rightPower);
            left.setPower(leftPower);

        }

        right.setPower(0);
        left.setPower(0);

    }

    /**Simple Four Wheel Drive Autonomous code to drive in a set direction for a set time.
     * @param driveDirection The direction for the robot to move.
     * @param SECONDS The time for the movement to occur over.*/
    public void timeAutoTwoWheelDrive(DriveDirection driveDirection, double power, double SECONDS) {

        double rightPower = power;
        double leftPower = power;

        switch (driveDirection) {

            case FORWARD:
                rightPower *= -1;
                leftPower *= -1;
                break;

            case BACKWARD:
                rightPower *= 1;
                leftPower *= 1;
                break;

            case STRAFE_LEFT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    rightPower *= -1;
                    leftPower *= 1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case STRAFE_RIGHT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    rightPower *= 1;
                    leftPower *= -1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case TURN_CLOCKWISE:
                rightPower *= -1;
                leftPower *= 1;
                break;

            case TURN_COUNTERCLOCKWISE:
                rightPower *= 1;
                leftPower *= -1;
                break;

            case DIAGONAL_FORWARD_LEFT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    rightPower *= 1;
                    leftPower *= 0;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case DIAGONAL_FORWARD_RIGHT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    rightPower *= 0;
                    leftPower *= 1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case DIAGONAL_BACKWARD_LEFT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    rightPower *= -1;
                    leftPower *= 0;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

            case DIAGONAL_BACKWARD_RIGHT:
                if (this.driveTrainType == DriveTrainType.MECANUM) {

                    rightPower *= 0;
                    leftPower *= -1;

                } else {

                    RobotLog.setGlobalErrorMsg("This only works on a mecanum drivebase.");

                }
                break;

        }

        this.timeAutoTwoWheelDrive(rightPower, leftPower, SECONDS);

    }

    /**Four Wheel Autonomous for a Mecanum Drivetrain using inches and degrees
     * @param INCHES_FB The inches forwards/backwards to drive (positive is forwards,
     *        negative is backwards)
     * @param INCHES_LR The inches left/right to drive (positive is right,
     *        negative is left)
     * @param DEGREES_TURN The degrees to turn (positive is clockwise,
     *        negative is counterclockwise)*/
    public void EncoderAutoMecanumDrive(double INCHES_FB, double INCHES_LR, double DEGREES_TURN, double SPEED, int time) {

        ElapsedTime timer = new ElapsedTime();

        if (DEGREES_TURN > 180) {

            DEGREES_TURN -= 360;

        }

        int frTargetPosition = fr.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                - (int) (this.COUNTS_PER_INCH * INCHES_LR)
                - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int brTargetPosition = br.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                + (int) (this.COUNTS_PER_INCH * INCHES_LR)
                - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int flTargetPosition = fl.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                + (int) (this.COUNTS_PER_INCH * INCHES_LR)
                + (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int blTargetPosition = bl.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                - (int) (this.COUNTS_PER_INCH * INCHES_LR)
                + (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);

        fr.setTargetPosition(frTargetPosition);
        br.setTargetPosition(brTargetPosition);
        fl.setTargetPosition(flTargetPosition);
        bl.setTargetPosition(blTargetPosition);

        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();

        fr.setPower(SPEED);
        br.setPower(SPEED);
        fl.setPower(SPEED);
        bl.setPower(SPEED);

        while ((fr.isBusy() || br.isBusy() || fl.isBusy() || bl.isBusy()) && timer.seconds() <= time) {

        }

        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();
        while (waitTimer.seconds() <= 0.125) {

        }

    }

    /**Four Wheel Autonomous for a Four Wheel Tank Drivetrain using inches and degrees
     * @param INCHES_FB The inches forwards/backwards to drive (positive is forwards,
     *        negative is backwards)
     * @param DEGREES_TURN The degrees to turn (positive is clockwise,
     *        negative is counterclockwise)*/
    public void EncoderAutoFourWheelTankDrive(double INCHES_FB, double DEGREES_TURN, double SPEED, int time) {

        ElapsedTime timer = new ElapsedTime();

        if (DEGREES_TURN > 180) {

            DEGREES_TURN -= 360;

        }

        int frTargetPosition = fr.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int brTargetPosition = br.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int flTargetPosition = fl.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                + (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int blTargetPosition = bl.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                + (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);

        fr.setTargetPosition(frTargetPosition);
        br.setTargetPosition(brTargetPosition);
        fl.setTargetPosition(flTargetPosition);
        bl.setTargetPosition(blTargetPosition);

        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();

        fr.setPower(SPEED);
        br.setPower(SPEED);
        fl.setPower(SPEED);
        bl.setPower(SPEED);

        while ((fr.isBusy() || br.isBusy() || fl.isBusy() || bl.isBusy()) && timer.seconds() <= time) {

        }

        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();
        while (waitTimer.seconds() <= 0.125) {

        }

    }

    /**Four Wheel Autonomous for a Two Wheel Drivetrain using inches and degrees
     * @param INCHES_FB The inches forwards/backwards to drive (positive is forwards,
     *        negative is backwards)
     * @param DEGREES_TURN The degrees to turn (positive is clockwise,
     *        negative is counterclockwise)*/
    public void EncoderAutoTwoWheelDrive(double INCHES_FB, double DEGREES_TURN, double SPEED, int time) {

        ElapsedTime timer = new ElapsedTime();

        if (DEGREES_TURN > 180) {

            DEGREES_TURN -= 360;

        }

        int rightTargetPosition = fr.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int leftTargetPosition = fl.getCurrentPosition()
                + (int) (this.COUNTS_PER_INCH * INCHES_FB)
                + (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);

        right.setTargetPosition(rightTargetPosition);
        left.setTargetPosition(leftTargetPosition);

        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();

        right.setPower(SPEED);
        left.setPower(SPEED);

        while ((left.isBusy() || right.isBusy()) && timer.seconds() <= time) {

        }

        right.setPower(0);
        left.setPower(0);

        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();
        while (waitTimer.seconds() <= 0.125) {

        }

    }

    public void driveEncoderRawTelemetry() {

        telemetryControl.addData("Front Left Position", fl.getCurrentPosition());
        telemetryControl.addData("Front Right Position", fr.getCurrentPosition());
        telemetryControl.addData("Back Left Position", bl.getCurrentPosition());
        telemetryControl.addData("Back Right Position", br.getCurrentPosition());

    }

}
