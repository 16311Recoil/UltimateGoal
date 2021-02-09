package org.firstinspires.ftc.teamcode.NopeRopeLibs.motion;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.EscapeAnalysis;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.PID;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
Contains methods from ACME's Roadrunner Mecanum Drive Class combined with 16311 Recoil methods
 */
public class Drivetrain extends com.acmerobotics.roadrunner.drive.MecanumDrive{

    public LinearOpMode opMode;
    public OpMode opMode_iterative;
    private DcMotorEx fl, fr, bl, br;
    private List<DcMotorEx> motors;
    private double multiplier = 1;
    private int multiCounter = 1;
    private double[] multipliers = {0.3, 0.55, 1};
    private String[] multipliersTelemetry = {"LOW POWER", "REGULAR POWER", "HIGH POWER"};

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    private boolean changeDpadDown = false;
    private boolean changeDpadUp = false;

    private enum State {
        FULL_SPEED,
        REGULAR_SPEED,
        LOW_SPEED,
        H_SCALE_POWER;
    }

    private double rawHeading;
    private State teleOpState;
    private PID[] pidCMotionontroller;

    private static final int PID_X = 0;
    private static final int PID_Y = 0;
    private static final int PID_THETA = 0;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private MecanumDrive.Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;
    private Pose2d lastPoseOnTurn;
    private List<LynxModule> allHubs;


    public Drivetrain(LinearOpMode opMode, TwoWheelLocalizer localizer) throws InterruptedException {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.opMode = opMode;

        //Roadrunner implementation

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = MecanumDrive.Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        fl = this.opMode.hardwareMap.get(DcMotorEx.class,"fl");
        fr = this.opMode.hardwareMap.get(DcMotorEx.class,"fr");
        bl = this.opMode.hardwareMap.get(DcMotorEx.class,"bl");
        br = this.opMode.hardwareMap.get(DcMotorEx.class,"br");


        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = Arrays.asList(fl, bl, br, fr);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        LynxModuleUtil.ensureMinimumFirmwareVersion(opMode.hardwareMap);

        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(localizer);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidCMotionontroller = new PID[]{new PID(), new PID(), new PID()};

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addLine("Drivetrain Init Completed1");
        opMode.telemetry.update();
    }

    public Drivetrain(OpMode opMode, TwoWheelLocalizer localizer) throws InterruptedException {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Drivetrain update");
        opMode_iterative.telemetry.update();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = MecanumDrive.Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        fl = this.opMode_iterative.hardwareMap.get(DcMotorEx.class,"fl");
        fr = this.opMode_iterative.hardwareMap.get(DcMotorEx.class,"fr");
        bl = this.opMode_iterative.hardwareMap.get(DcMotorEx.class,"bl");
        br = this.opMode_iterative.hardwareMap.get(DcMotorEx.class,"br");

        teleOpState = State.REGULAR_SPEED;

        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = Arrays.asList(fl, bl, br, fr);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        LynxModuleUtil.ensureMinimumFirmwareVersion(opMode_iterative.hardwareMap);

        for (LynxModule module : opMode_iterative.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        setLocalizer(localizer);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode_iterative.telemetry.addLine("Drivetrain Complete");
        opMode_iterative.telemetry.update();


    }

    /* ============================================== UTILITY METHODS ==============================================================*/
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        fl.setPower(v);
        fr.setPower(v1);
        bl.setPower(v2);
        br.setPower(v3);
    }
    public void setAllMotors(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void turn(double power, boolean turnRight) {
        int turnMultiplier = -1;
        if (turnRight) {
            turnMultiplier = 1;
        }
        fl.setPower(power * turnMultiplier);
        fr.setPower(-power * turnMultiplier);
        bl.setPower(power * turnMultiplier);
        br.setPower(-power * turnMultiplier);
    }

    public double getDistance() {
        return 0;
        //placeholder for odom method in sensors class - REMOVE AFTER REAL METHOD IMPLEMENTED
        //Ctrl F marker for methods that need to be updated
    }

    public double getAngle() {
        return 0;
        //placeholder for odom method in sensors class - REMOVE AFTER REAL METHOD IMPLEMENTED
        //Ctrl F marker for methods that need to be updated
    }

    public void moveForward(double distance, double power, double timeout) {
        double currentPos = getDistance();
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < timeout && currentPos < distance) {
            currentPos = getDistance();
            setAllMotors(power);
        }
        setAllMotors(0);
    }

    public void turnTo(double power, double desiredAngle, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        double currentAngle = getAngle(); //radians
        if (Math.abs(desiredAngle - currentAngle) > Math.PI) {
            desiredAngle += 360;
        }
        boolean turnRight = ((currentAngle - desiredAngle) < (desiredAngle - currentAngle));
        if (turnRight) {
            turn(power, true);
            while ((currentAngle > desiredAngle) && (timer.seconds() < timeout)) {
            }
        } else {
            turn(power, false);
            while ((desiredAngle > currentAngle) && (timer.seconds() < timeout)) {
            }
        }
        setAllMotors(0);
    }

    public void move(double x, double y, double z) {
        /*
        if (teleOpState.equals(State.H_SCALE_POWER))
            multiplier = lowSpeedAccuracyFunction(v_d);

         */
        fl.setPower(multiplier * Range.clip(y + x - z, -1, 1));
        fr.setPower(multiplier * Range.clip(y - x + z, -1, 1));
        bl.setPower(multiplier * Range.clip(y - x - z, -1, 1));
        br.setPower(multiplier * Range.clip(y + x + z, -1, 1));
    }
    /*
    public void move(double v_d, double netTheta, double z){
        fl.setPower( (v_d * (Math.sin((netTheta)))) + v_d * Math.cos(netTheta) - z);
        fr.setPower((v_d * (Math.sin((netTheta)))) - v_d * Math.cos(netTheta) + z);
        bl.setPower((v_d * (Math.sin((netTheta)))) - v_d * Math.cos(netTheta) - z);
        br.setPower(v_d*Math.sin(netTheta) + (v_d * (Math.cos((netTheta)))) + z);
    }


     */
    @Override
    protected double getRawExternalHeading() {
        return rawHeading;
    }

    public void setRawHeading(double rawHeading) {
         this.rawHeading = rawHeading;
    }

    public void moveToPositionPID(double x, double y, double theta, double timeout, double[][] constants){
        ElapsedTime timer = new ElapsedTime();
        Pose2d pose = getPoseEstimate();
        double x_i = pose.getX();
        double y_i = pose.getY();
        double theta_i = pose.getHeading();

        boolean loopCondition = Math.abs(x_i - x) >= 0.05 ||
                                Math.abs(y_i - y) >= 0.05 ||
                                Math.abs(theta_i - theta) >= 0.05 &&
                                timer.milliseconds() <= timeout;

        pidCMotionontroller[PID_X].setConstants(constants[0][0], constants[0][1], constants[0][2], x);
        pidCMotionontroller[PID_Y].setConstants(constants[1][0], constants[1][1], constants[1][2], y);
        pidCMotionontroller[PID_THETA].setConstants(constants[2][0], constants[2][1], constants[2][2], theta);

        while (loopCondition){
            double p_x = pidCMotionontroller[PID_X].loop(x_i, timer.milliseconds());
            double p_y = pidCMotionontroller[PID_Y].loop(y_i, timer.milliseconds());
            double p_theta = pidCMotionontroller[PID_THETA].loop(theta_i, timer.milliseconds());

            move(p_x, p_y, p_theta);

            x_i = pose.getX();
            y_i = pose.getY();
            theta_i = pose.getHeading();

            double x_error = x_i - x;
            double y_error = y_i - y;
            double theta_error = theta_i - theta;

            loopCondition = Math.abs(x_error) > 0.05 ||
                    Math.abs(y_error) > 0.05 ||
                    Math.abs(theta_error) > 0.05 &&
                    timer.milliseconds() < timeout;


        }


    }




    //================================================================= Tele-Op Methods ===============================================================//

    public void moveTelop(double x, double y, double z) {
        double v_d = Math.hypot(x, y) * Math.signum(x) * Math.signum(y);
        double netTheta = Math.atan2(x, y);
        if (v_d < 0.05)
            v_d = 0;
        if (v_d > 0.95)
            v_d = 1;

        if (teleOpState.equals(State.FULL_SPEED))
            multiplier = 1;
        if (teleOpState.equals(State.REGULAR_SPEED))
            multiplier = 0.5;
        if (teleOpState.equals(State.LOW_SPEED))
            multiplier = .25;
        /*
        if (teleOpState.equals(State.H_SCALE_POWER))
            multiplier = lowSpeedAccuracyFunction(v_d);

         */

        toggleSpeed();

        fl.setPower(multiplier * Range.clip(y + x - z, -1, 1));
        fr.setPower(multiplier * Range.clip(y - x + z, -1, 1));
        bl.setPower(multiplier * Range.clip(y - x - z, -1, 1));
        br.setPower(multiplier * Range.clip(y + x + z, -1, 1));
    }



    public double lowSpeedAccuracyFunction(double x){
        return 1.442 * x - 0.721 * Math.pow(x, 2) + 0.480667 * Math.pow(x, 3) - 0.3605 * Math.pow(x, 4) - 0.244033 * Math.pow(x, 6);
    }




    //Have to make sure to add these controls in the gamepad class
    public void toggleSpeed() {
        if ((opMode_iterative.gamepad1.dpad_down && !changeDpadDown) && multiCounter > 0) {
            multiCounter--;
        }
        else if ((opMode_iterative.gamepad1.dpad_up && !changeDpadUp) && multiCounter < 2) {
            multiCounter++;
        }
        multiplier = multipliers[multiCounter];


        changeDpadDown = opMode_iterative.gamepad1.dpad_down;
        changeDpadUp = opMode_iterative.gamepad1.dpad_up;

    }


   // ======================================= RR METHODS TO IMPLEMENT =================================
    /*
    the following methods have been implemented from ACME RoadRunner
     */
   public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
       return new TrajectoryBuilder(startPose, constraints);
   }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        turnStart = clock.seconds();
        mode = MecanumDrive.Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = MecanumDrive.Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = MecanumDrive.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = MecanumDrive.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != MecanumDrive.Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = fl.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }



}
