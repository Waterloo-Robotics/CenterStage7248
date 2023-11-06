package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class H2OLooAuto extends LinearOpMode {

    public H2OLooAuto() {}
    public TelemetryControl telemetryControl;

    public DriveTrain driveTrain;
    public OdometryControl odometryControl;
    public DriveTrain driveTrain;
    public AttachmentControl attachmentControl;

    public CameraControl cameraControl;

    public void runOpMode() {

        telemetryControl = new TelemetryControl(telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetryControl);
//        odometryControl = new OdometryControl(hardwareMap, telemetryControl);
        attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, gamepad1, gamepad2);
//        cameraControl = new CameraControl(hardwareMap, telemetryControl);

        this.opModeInit();

        waitForStart();

        this.opModePeriodic();

    }

    abstract public void opModeInit();

    abstract public void opModePeriodic();

}
