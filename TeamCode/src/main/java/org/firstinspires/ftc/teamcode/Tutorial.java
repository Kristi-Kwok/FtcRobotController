package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous
public class Tutorial extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Hello", "Kristi");
    }

    @Override
    public void loop() {

    }
}
