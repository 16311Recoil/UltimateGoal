package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
        (name = "EncodersTest", group = "Testing")
public class EncoderTest extends LinearOpMode {

    DcMotor motor;
    FtcDashboard dashboard;
    private TelemetryPacket packet;
    private double targetPos;


    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "screwMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()){
            packet = new TelemetryPacket();
            double currPos = motor.getCurrentPosition();

            if (gamepad1.a){
                targetPos += 750;
            }
            if (gamepad1.b && currPos < targetPos){
                motor.setPower(0.8);
            }

            packet.put("Encoder Value", currPos);
            packet.put("Error", targetPos - currPos);
            telemetry.addData("Encoder Value", currPos);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

    }
}
