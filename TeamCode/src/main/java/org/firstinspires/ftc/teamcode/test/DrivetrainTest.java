package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.Drivetrain;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.TwoWheelLocalizer;


@Autonomous(name="DriveTest", group="Linear Opmode")
public class DrivetrainTest extends LinearOpMode {
    private Drivetrain drivetraint;
    private ElapsedTime timer;
    private double power = 0.5;
    private double angle = 0;
    @Override


    public void runOpMode() throws InterruptedException {
        drivetraint = new Drivetrain(this,new TwoWheelLocalizer(this.hardwareMap, new Sensors(this)));
        timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (timer.milliseconds() < 3000){

            drivetraint.move(0, 0, 0.3);

        }
        timer.reset();

        drivetraint.setAllMotors(0);
    }
}
