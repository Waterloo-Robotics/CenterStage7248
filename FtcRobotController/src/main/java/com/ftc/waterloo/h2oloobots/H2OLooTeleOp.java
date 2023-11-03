package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class H2OLooTeleOp extends LinearOpMode {

    public H2OLooTeleOp() {}
    public TelemetryControl telemetryControl;

    public DriveTrain driveTrain;
    public AttachmentControl attachmentControl;

    public CameraControl cameraControl;

    public void runOpMode() {

        telemetryControl = new TelemetryControl(telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetryControl);
        attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, gamepad1, gamepad2);
        cameraControl = new CameraControl(hardwareMap, telemetryControl);

        this.opModeInit();

        waitForStart();

        while (opModeIsActive()) {
            this.opModePeriodic();
            telemetryControl.update();
        }

    }

    abstract public void opModeInit();

    abstract public void opModePeriodic();

}
