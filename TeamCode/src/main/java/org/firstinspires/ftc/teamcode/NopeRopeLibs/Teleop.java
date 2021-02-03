package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.Drivetrain;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.TrackingWheelLocalizer;

import java.util.Arrays;
import java.util.HashMap;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Iterative Opmode")

public class Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private NopeRope robot;
    private double power = 0.75;
    private boolean changeRB = false;
    private boolean changeRB2 = false;
    private final int revolution = 754;
    private int targetPos = 0;

    private enum TransitionState{
        IDLE,
        SCREW_TO_POSITION_UP,
        SCREW_TO_POSITION_DOWN
    }

    private TransitionState state;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        try {
            robot = new NopeRope(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // Tell the driver that initialization is complete.
        state = TransitionState.IDLE;
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
        runtime.reset();
        robot.getShooter().setOut();
        robot.getShooter().getScrewMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getShooter().getScrewMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        int currPos = robot.getShooter().getScrewMotor().getCurrentPosition();

        if (((gamepad1.right_bumper && changeRB) || (gamepad2.right_bumper && changeRB2)) && state == TransitionState.IDLE) {
            state = TransitionState.SCREW_TO_POSITION_UP;
            targetPos += revolution;
        }
        if (state == TransitionState.SCREW_TO_POSITION_UP){
            if (currPos < targetPos)
                robot.getShooter().getScrewMotor().setPower(1);
            else{
                robot.getShooter().getScrewMotor().setPower(0);
                state = TransitionState.IDLE;
            }

        }
        changeRB = gamepad1.right_bumper;
        changeRB2 = gamepad2.right_bumper;
        robot.teleOpControls();
       // robot.getShooter().getScrewMotor().getCurrentPosition() > 0
        telemetry.addData("Screw Pos",robot.getShooter().getScrewMotor().getCurrentPosition());
        telemetry.addData("Screw Target", targetPos);

        telemetry.update();
        //dt.moveTelop(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

