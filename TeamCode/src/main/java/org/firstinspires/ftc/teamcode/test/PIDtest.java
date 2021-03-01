package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;

@TeleOp(name = "PID TEST", group = "Testing")
public class PIDtest extends LinearOpMode {
    NopeRope robot;
    FtcDashboard dash;
    double[][] CONSTANTS = new double[3][3];

    private static final int X = 0;
    private static final int Y = 1;
    private static final int Z = 2;

    private static final int kp = 0;
    private static final int ki = 1;
    private static final int kd = 2;

    private static final double FORWARD = Math.PI/2;
    private static final double BACKWARD = 3 * Math.PI/2;
    private static final double LEFT = Math.PI;
    private static final double RIGHT = 2 * Math.PI;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NopeRope(this);
        dash = FtcDashboard.getInstance();
        robot.getDrivetrain().setDashboard(dash);

        /*
        Trajectory myTrajectory = robot.getDrivetrain().trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();


         */


        waitForStart();



        //robot.getDrivetrain().moveToPositionPID(30.0, 0,  3 * Math.PI/2, 8, CONSTANTS);

        //test movement
        //diagnol strafe

        CONSTANTS[X][kp] = 0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0;

        CONSTANTS[Y][kp] = 0.;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.05;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;

        //robot.getDrivetrain().turnTo(0.5, CONSTANTS);



        /*// MOVEMENT 1 ==============================================================================================
        //forward to stack
        CONSTANTS[X][kp] = 0.90 / 70.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.1;

        CONSTANTS[Y][kp] = 0;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        robot.getDrivetrain().moveToPositionPID(75.0, 0,  BACKWARD, 4 * (1000), CONSTANTS); // dec time by 1?



        // MOVEMENT 2 ==============================================================================================
        // strafe to see the starter stack
        CONSTANTS[X][kp] = 0.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.00;

        CONSTANTS[Y][kp] = 0.9 / 20;
        CONSTANTS[Y][ki] = 0.;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.035;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0.25;


        robot.getDrivetrain().moveToPositionPID(0.0, -25,  LEFT, 3 * (1000), CONSTANTS); // dec time by 1?


        //Movement 3
        //strafe back
        CONSTANTS[X][kp] = 0.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.00;

        CONSTANTS[Y][kp] = 0.9 / 20;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.035;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0.15;


        robot.getDrivetrain().moveToPositionPID(0.0, 5,  RIGHT, 3 * (1000), CONSTANTS);

        //Movement 4
        //backward to box
        CONSTANTS[X][kp] = 0.90 / 25;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.1;

        CONSTANTS[Y][kp] = 0;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.02;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        robot.getDrivetrain().moveToPositionPID(95.0, 0,  BACKWARD, 4 * (1000), CONSTANTS);

        //Movement 5
        //back to original
        CONSTANTS[X][kp] = 0.95 / 90.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.04;

        CONSTANTS[Y][kp] = 0;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.02;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        robot.getDrivetrain().moveToPositionPID(-3.25, 0,  FORWARD, 5 * (1000), CONSTANTS);

        //Movement 6
        //Strafe left to grab
        CONSTANTS[X][kp] = 0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0;

        CONSTANTS[Y][kp] = 0.6 / 30;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.035;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0.25;


        robot.getDrivetrain().moveToPositionPID(0, -30,  LEFT, 3 * (1000), CONSTANTS);
        telemetry.addData("Encoders", robot.getSensors().getEncoders());


        //Movement 7
        //Strafe right to grab
        CONSTANTS[X][kp] = 0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0;

        CONSTANTS[Y][kp] = 0.9 / 30;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.035;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0.15;


        robot.getDrivetrain().moveToPositionPID(0, 5,  RIGHT, 3 * (1000), CONSTANTS);
        telemetry.addData("Encoders", robot.getSensors().getEncoders());


        //movement 8
        //move to box
        CONSTANTS[X][kp] = 0.90 / 80.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.1;

        CONSTANTS[Y][kp] = 0;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.02;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        robot.getDrivetrain().moveToPositionPID(100.0, 0,  BACKWARD, 4.5 * (1000), CONSTANTS);


        //movement 9
        //park
        CONSTANTS[X][kp] = 0.90 / 80.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.1;

        CONSTANTS[Y][kp] = 0;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.02;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        robot.getDrivetrain().moveToPositionPID(-70.0, 0,  FORWARD, 3 * (1000), CONSTANTS);




        // STRAFE RIGHT 48 INCHES
        /*

        double targetDistance = 48.0;

        CONSTANTS[X][kp] = 0;
        CONSTANTS[X][ki] = 0;
        CONSTANTS[X][kd] = 0;

        CONSTANTS[Y][kp] = 0.9 / targetDistance; //0.8 / targetDistance;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0.03;
        CONSTANTS[Z][ki] = 0.000;
        CONSTANTS[Z][kd] = 0.13;

        robot.getDrivetrain().moveToPositionPID(0, targetDistance, 2* Math.PI, 100000, CONSTANTS);
        //robot.getDrivetrain().turnTo(0.3, Math.PI / 2, 4);

         */

    }

}
