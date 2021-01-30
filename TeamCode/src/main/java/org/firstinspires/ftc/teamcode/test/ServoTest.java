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

    @Override
    public void runOpMode() throws InterruptedException {
        armRotater = hardwareMap.servo.get("ringPusher");
        //armRotater.setDirection(Servo.Direction.REVERSE);
        armRotater.setPosition(0);
        waitForStart();

        while (opModeIsActive()){
            //rFang.setPosition(0.32);
            //lFang.setPosition(0.32);
            armRotater.setPosition(0.22);
            //    pincher.setPosition(1);
            //telemetry.addData("Position", rFang.getPosition());
        }




    }
}