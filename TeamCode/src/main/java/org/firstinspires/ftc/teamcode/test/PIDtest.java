package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;

public class PIDtest extends LinearOpMode {
    NopeRope robot;
    FtcDashboard dash;
    TelemetryPacket packet;
    double[][] CONSTANTS = new double[3][3];

    private static final int X = 0;
    private static final int Y = 1;
    private static final int Z = 2;

    private static final int kp = 0;
    private static final int ki = 1;
    private static final int kd = 2;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NopeRope(this);
        dash = FtcDashboard.getInstance();

        CONSTANTS[X][kp] = 0.5 / 24;
        CONSTANTS[X][ki] = 0;
        CONSTANTS[X][kd] = 0;

        CONSTANTS[Y][kp] = 0;
        CONSTANTS[Y][ki] = 0;
        CONSTANTS[Y][kd] = 0;

        CONSTANTS[Z][kp] = 0;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0;


        waitForStart();

        robot.getDrivetrain().moveToPositionPID(24, 0, 0, 5, CONSTANTS);

    }
}
