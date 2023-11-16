package org.firstinspires.ftc.teamcode.tests;

import com.ftc.waterloo.h2oloobots.H2OLooTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class EncoderTest extends H2OLooTeleOp {

    @Override
    public void opModeInit() {

    }

    @Override
    public void opModePeriodic() {

        driveTrain.driveEncoderRawTelemetry();

    }

}
