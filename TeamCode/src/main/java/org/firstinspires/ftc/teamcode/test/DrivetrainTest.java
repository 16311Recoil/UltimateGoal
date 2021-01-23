package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.Drivetrain;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.TrackingWheelLocalizer;


@Autonomous(name="DriveTest", group="Linear Opmode")
public class DrivetrainTest extends LinearOpMode {
    private Drivetrain drivetraint;
    private ElapsedTime timer;
    private double power = 0.5;
    private double angle = 0;
    @Override


    public void runOpMode() throws InterruptedException {
        drivetraint = new Drivetrain(this,new TrackingWheelLocalizer(this.hardwareMap));
        timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (timer.milliseconds() < 2000){

            drivetraint.move(power, Math.toRadians(90), 0);

        }
        timer.reset();

        while (timer.milliseconds() < 500){

            drivetraint.setAllMotors(0);

        }
        timer.reset();

        while (timer.milliseconds() < 2000){

            drivetraint.move(power, Math.toRadians(180), 0);

        }
        timer.reset();

        while (timer.milliseconds() < 500){

            drivetraint.setAllMotors(0);

        }
        timer.reset();

        while (timer.milliseconds() < 2000){

            drivetraint.move(power, Math.toRadians(270), 0);

        }
        timer.reset();

        while (timer.milliseconds() < 500){

            drivetraint.setAllMotors(0);

        }
        timer.reset();

        while (timer.milliseconds() < 2000){

            drivetraint.move(power, Math.toRadians(0), 0);

        }
        timer.reset();


        while (timer.milliseconds() < 500){

            drivetraint.setAllMotors(0);

        }
        timer.reset();

        drivetraint.setAllMotors(0);
    }
}
