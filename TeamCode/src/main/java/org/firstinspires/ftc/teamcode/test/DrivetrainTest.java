package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.Drivetrain;

public class DrivetrainTest extends LinearOpMode {

    private Drivetrain drivetraint;
    private ElapsedTime timer;
    private double power = 0.5;
    private double angle = 0;
    @Override


    public void runOpMode() throws InterruptedException {
        drivetraint = new Drivetrain(this, timer);
        timer = new ElapsedTime();
        waitForStart();

        while (timer.milliseconds() > 2000){
            drivetraint.moveTelop(power * Math.cos(angle), power * Math.sin(angle), 0);

        }

        drivetraint.setAllMotors(0);
    }
}
