package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.sql.Time;

@Autonomous
        (name = "Distance Sensors Test", group = "Test")
public class DistanceSensorsTest extends LinearOpMode {
    private DistanceSensor sensorRange;
    private DistanceSensor sensorRange2;


    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "transitionDS");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "shooterDS");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        int loopCount = 0;
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("transitionDS-Checkpoint", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("shooterDS-Checkpoint", String.format("%.01f in", sensorRange2.getDistance(DistanceUnit.MM)));
            telemetry.addData("loop speed loops/ms", (loopCount * 1.0/timer.milliseconds()));

            telemetry.update();
        }
    }
}
