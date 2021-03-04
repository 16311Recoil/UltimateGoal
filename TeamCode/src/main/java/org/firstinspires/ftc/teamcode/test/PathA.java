package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;
import org.firstinspires.ftc.teamcode.VistionTesting.VisionTest;
import org.firstinspires.ftc.teamcode.VistionTesting.VisonTestWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Path A", group = "Testing")
public class PathA extends LinearOpMode {
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

    OpenCvCamera webcam;
    FtcDashboard dashboard;
    private int ringStackSize;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NopeRope(this);
        dash = FtcDashboard.getInstance();
        robot.getDrivetrain().setDashboard(dash);
        ringStackSize = -1;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        dashboard = FtcDashboard.getInstance();

        webcam.setPipeline(new VisonTestWebcam.SamplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        robot.getShooter().toggleWobbleGrabber(true);
        robot.getShooter().toggleWobblePivot(false);


        /*
        Trajectory myTrajectory = robot.getDrivetrain().trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();


         */


        waitForStart();




        // MOVEMENT 1 ==============================================================================================
        //forward to stack
        CONSTANTS[X][kp] = 0.90 / 70.0;
        CONSTANTS[X][ki] = 0.000;
        CONSTANTS[X][kd] = 0.08;

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
        CONSTANTS[Y][kd] = 0.03;

        CONSTANTS[Z][kp] = 0.035;
        CONSTANTS[Z][ki] = 0;
        CONSTANTS[Z][kd] = 0.25;


        robot.getDrivetrain().moveToPositionPID(0.0, -26,  LEFT, 3.5 * (1000), CONSTANTS); // dec time by 1?
        robot.getShooter().toggleWobblePivot(true);
        Thread.sleep(600);
        robot.getShooter().toggleWobbleGrabber(false);



        while (opModeIsActive() && ringStackSize < 0) {

            TelemetryPacket p = new TelemetryPacket();
            ringStackSize = VisonTestWebcam.SamplePipeline.stackSize;
            telemetry.addData("RingStackSize", ringStackSize);
            p.put("RingStackSize", ringStackSize);
            dashboard.sendTelemetryPacket(p);
            dashboard.startCameraStream(webcam, 30);
        }

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


        robot.getDrivetrain().moveToPositionPID(0.0, -9,  RIGHT, 3 * (1000), CONSTANTS);




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


        robot.getDrivetrain().moveToPositionPID(0, 0,  FORWARD, 5 * (1000), CONSTANTS);

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

        //robot.getShooter().toggleWobbleGrabber(false);
        //robot.getShooter().toggleWobblePivot(false);


        //Movement 7
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

        //robot.getShooter().toggleWobblePivot(true);
        //robot.getShooter().toggleWobbleGrabber(true);











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
