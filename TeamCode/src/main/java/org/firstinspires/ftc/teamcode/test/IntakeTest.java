package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.Intake;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;

@TeleOp(name="IntakeTest", group="Iterative Opmode")

public class IntakeTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private NopeRope robot;
    private double power = 0.5;
    private double increment = 0.1;
    //private Intake intake;//do i need this if i already have NopeRope
    private Drivetrain dt;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        robot = new NopeRope(this);
        //intake = new Intake(this, null);
        try {
            dt = new Drivetrain(this, null);
        } catch (InterruptedException e) {
            telemetry.addLine("NOT WORJ");
            telemetry.update();
            e.printStackTrace();
        }

        // Tell the driver that initialization is complete.
        //telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        ElapsedTime timer = new ElapsedTime();
        double angle = 0;

        while (timer.milliseconds() < 2000){
            dt.moveTelop(power * Math.cos(angle), power * Math.sin(angle), 0);

        }
    }

    @Override
    public void loop() {

    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}