package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.MecanumDrive;

import java.util.ArrayList;

public class MecanumDriveRRTest extends OpMode {

    MecanumDrive dt;

    @Override
    public void init() {
        dt = new MecanumDrive(this.hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addLine(String.valueOf((ArrayList)dt.getWheelPositions()));
        telemetry.update();
    }
}
