package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private boolean changeB = false;
    private boolean changeX = false;
    private boolean changeY = false;
    //private Intake intake;//do i need this if i already have NopeRope
    private Intake in;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


//        robot = new NopeRope();
        //intake = new Intake(this, null);
        in = new Intake(this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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

    }

    @Override
    public void loop() {
        ElapsedTime timer = new ElapsedTime();
        double angle = 0;
        if (gamepad1.a){
            in.setPower(power);
        }
        else {
            in.setPower(0);
        }
        if (gamepad1.b && !changeB && power != 1){
            power += 0.05;
        }
        if (gamepad1.x && !changeX && power != -1){
            power -= 0.05;
        }
        if (gamepad1.y && !changeY){
            power *= -1;
        }
        changeB = gamepad1.b;
        changeX = gamepad1.x;
        changeY = gamepad1.y;
        telemetry.addData("Power", power);
        telemetry.update();
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
