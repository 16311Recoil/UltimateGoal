package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.MecanumDrive;

import java.util.ArrayList;
@TeleOp
        (name = "Encoders Test")
public class MecanumDriveRRTest extends OpMode {

    NopeRope robot;

    @Override
    public void init() {
        try {
            robot = new NopeRope(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    @Override
    public void loop() {
        telemetry.addLine(robot.getSensors().getWheelPos().toString());
        telemetry.update();
    }
}
