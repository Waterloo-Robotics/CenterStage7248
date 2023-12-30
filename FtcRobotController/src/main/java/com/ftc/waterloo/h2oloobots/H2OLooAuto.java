package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class H2OLooAuto extends LinearOpMode {

    public H2OLooAuto() {}
    public CameraControl.Alliance alliance;
    public TelemetryControl telemetryControl;

    public DriveTrain driveTrain;
    public OdometryControl odometryControl;
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

    public void initCamera(CameraControl.Alliance alliance) {

        this.alliance = alliance;
        this.cameraControl = new CameraControl(hardwareMap, telemetryControl, this.alliance, gamepad1, gamepad2);

    }

    abstract public void opModeInit();

    abstract public void opModePeriodic();

}
