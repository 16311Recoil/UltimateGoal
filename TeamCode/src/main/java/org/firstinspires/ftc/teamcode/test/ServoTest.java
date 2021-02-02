package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
        (name = "ServoTest", group = "Controlled")
//@Disabled
public class ServoTest extends LinearOpMode {

    private Servo armRotater;
    private boolean changeDpadUp = false;
    private boolean changeDpadDown = false;
    private double armPos = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {
        armRotater = hardwareMap.servo.get("wobblePivot");
        armRotater.setDirection(Servo.Direction.FORWARD);
        //armRotater.setDirection(Servo.Direction.REVERSE);
        armRotater.setPosition(0.62);

        waitForStart();

        while (opModeIsActive()) {
            armRotater.setPosition(0.95);
        }
            //    pincher.setPosition(1);
            //telemetry.addData("Position", rFang.getPosition());





    }
}