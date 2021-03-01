package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.pidMovements.PID;

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
    private double power = 0.0;
    private boolean changeRB = true;
    private boolean changeRB2 = false;
    private final int revolution = 754;
    private int targetPos = 0;
    private boolean changeLB = false;
    private boolean changeLB2 = false;
    private FtcDashboard dashboard;
    private Telemetry dashboardTelem;
    private int currPos = 0;
    private PID screwPID;
    private final double TIME_THRESHOLD = 550;
    private boolean dpadRight = false;
    private boolean changeDpadLeft = false;


    private enum TransitionState{
        IDLE,
        SCREW_TO_POSITION_UP,
        SCREW_ACCURACY,
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
        dashboard = FtcDashboard.getInstance();
        dashboardTelem = dashboard.getTelemetry();
        runtime.reset();
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
        robot.getShooter().getScrewMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getShooter().getScrewMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){

        currPos = robot.getShooter().getScrewMotor().getCurrentPosition();
        TelemetryPacket packet = new TelemetryPacket();




        if (gamepad2.right_bumper && !changeRB2 && state == TransitionState.IDLE) {
            state = TransitionState.SCREW_TO_POSITION_UP;
            runtime.reset();
            targetPos += revolution;

        }

        if (gamepad2.dpad_right && !dpadRight && state == TransitionState.IDLE ) {
            state = TransitionState.SCREW_ACCURACY;
        }

        if(gamepad2.left_bumper && !changeLB2 && state == TransitionState.IDLE) {
            state = TransitionState.SCREW_TO_POSITION_DOWN;
            targetPos -= revolution;
        }

        if(state == TransitionState.SCREW_ACCURACY){
            if (gamepad2.left_bumper){
                robot.getShooter().getScrewMotor().setPower(-0.6);
            }
            else if (gamepad2.right_bumper){
                robot.getShooter().getScrewMotor().setPower(0.6);
            }
            else if (gamepad2.dpad_left && !changeDpadLeft)
                state = TransitionState.SCREW_TO_POSITION_UP;
            else
                robot.getShooter().setScrewPower(0);
        }
        if (state == TransitionState.SCREW_TO_POSITION_UP){
            if (currPos <= targetPos){
                robot.getShooter().getScrewMotor().setPower(0.8);
            }
            else {
                robot.getShooter().getScrewMotor().setPower(0);
                state = TransitionState.IDLE;
            }

        }

        if (state == TransitionState.SCREW_TO_POSITION_DOWN){
            if (currPos >= targetPos)
                robot.getShooter().getScrewMotor().setPower(-1);
            else{
                robot.getShooter().getScrewMotor().setPower(0);
                state = TransitionState.IDLE;
            }

        }

        changeRB2 = gamepad2.right_bumper;
        changeLB2 = gamepad2.left_bumper;
        dpadRight = gamepad2.dpad_right;
        changeDpadLeft = gamepad2.dpad_left;
        packet.put("Power", power);
        packet.put("Screw Pos", currPos);
        packet.put("Screw Target", targetPos);
        packet.put("error", targetPos - currPos);
        dashboard.sendTelemetryPacket(packet);
        robot.teleOpControls();
       // robot.getShooter().getScrewMotor().getCurrentPosition() > 0
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

