package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.PID;

@Autonomous(name = "PID TEST", group = "Testing")
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



        // MOVEMENT 1 ==============================================================================================
        //move to scan the starter stack
        CONSTANTS[X][kp] = 0.90 / 60.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.1;

        CONSTANTS[Y][kp] = 0;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        robot.getDrivetrain().moveToPositionPID(70.0, 0,  BACKWARD, 6 * (1000), CONSTANTS); // dec time by 1?



        // MOVEMENT 2 ==============================================================================================
        /* strafe to see the starter stack
        CONSTANTS[X][kp] = 0.97 / 60.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.00;

        CONSTANTS[Y][kp] = 0;;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        robot.getDrivetrain().moveToPositionPID(60.0, 0,  FORWARD, 6 * (1000), CONSTANTS); // dec time by 1?

         */


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
