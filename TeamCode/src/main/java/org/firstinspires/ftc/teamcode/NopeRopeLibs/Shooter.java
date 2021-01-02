package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Shooter{
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private DcMotor shooterMotor; //Outtake Wheels
    private DcMotor rotationMotor; //Larger Rotation Device
    private DcMotor screwMotor; //Motor to power the screws

    private Servo ringPusher; //Pushes the ring into the outtake
    private Servo angleChanger; //Adjusts the angle of the ramp/shooter


    private boolean transitionValid;


    private boolean shooterValid;

    private final double SERVO_POSITION_TO_ANGLE_FACTOR = 0; //Test for later
    private final double PUSH_OUT = 0;
    private final double PUSH_IN = 0;
    private boolean push = true;

    private double rampAngleTeleOP = 0;



    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.addLine("Shooter Init Started");
        opMode.telemetry.update();

        //Motors
        shooterMotor = this.opMode.hardwareMap.dcMotor.get("shooterMotor");
        rotationMotor = this.opMode.hardwareMap.dcMotor.get("rotationMotor");
        screwMotor = this.opMode.hardwareMap.dcMotor.get("screwMotor");


        //Servos
        ringPusher = this.opMode.hardwareMap.servo.get("ringPusher");
        angleChanger = this.opMode.hardwareMap.servo.get("angleChanger");



        rotationMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        screwMotor.setDirection(DcMotor.Direction.FORWARD);


        ringPusher.setDirection(Servo.Direction.FORWARD);
        angleChanger.setDirection(Servo.Direction.FORWARD);

        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        screwMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        opMode.telemetry.addLine("Shooter Init Completed");
        opMode.telemetry.update();

    }

    public Shooter(OpMode opMode) {
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Shooter Init Started");
        opMode_iterative.telemetry.update();

        //Motors
        shooterMotor = this.opMode.hardwareMap.dcMotor.get("shooterMotor");
        rotationMotor = this.opMode.hardwareMap.dcMotor.get("rotationMotor");
        screwMotor = this.opMode.hardwareMap.dcMotor.get("screwMotor");


        //Servos
        ringPusher = this.opMode.hardwareMap.servo.get("ringPusher");
        angleChanger = this.opMode.hardwareMap.servo.get("angleChanger");



        rotationMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        screwMotor.setDirection(DcMotor.Direction.FORWARD);

        ringPusher.setDirection(Servo.Direction.FORWARD);
        angleChanger.setDirection(Servo.Direction.FORWARD);

        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        screwMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        opMode.telemetry.addLine("Shooter Init Completed");
        opMode.telemetry.update();

    }


    public void setRotationPower(double power){
        rotationMotor.setPower(power);
    }

    public void setShooterPower (double power){
        shooterMotor.setPower(power);
    }

    public void setScrewPower (double power){
        screwMotor.setPower(power);
    }

    public void shooterTime(double power, double time){
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time){
            setShooterPower(power);
        }
        setShooterPower(0);
    }

    public void rotationTime(double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            setRotationPower(power);
        }
        setRotationPower(0);
    }

    public void screwTime(double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            setScrewPower(power);
        }
        setScrewPower(0);
    }

    /*public void rotationEncoder(double power, double angle) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            setRotationPower(power);
        }
        setRotationPower(0);
    }*/

     public boolean pivotRotation (double currentAngle, double desiredAngle){
        desiredAngle = Math.toRadians(desiredAngle);
        currentAngle = Math.toRadians(currentAngle);

        if (Math.abs(desiredAngle - currentAngle) > Math.PI) {
           if (Math.min(desiredAngle, currentAngle) == desiredAngle){
               desiredAngle += (2 * Math.PI);
           } else{
               currentAngle += (2 * Math.PI);
           }
        }
       return ((currentAngle-desiredAngle) > (desiredAngle-currentAngle));
    }


    public void setShooterAngle(double angle){
        angleChanger.setPosition(angle * SERVO_POSITION_TO_ANGLE_FACTOR);
    }

    public void togglePusher(){
        if (push){
            ringPusher.setPosition(PUSH_OUT);
        }
        else {
            ringPusher.setPosition(PUSH_IN);
        }
        push = !push;
    }

    public void pushRingUp(double power) {
        while (!isTransitionValid()){
            screwMotor.setPower(power);
        }
        screwMotor.setPower(0);
    }

    public boolean discsFull() {
        return isTransitionValid();
    }

    public boolean discAtTop() { //distance sensor
        return isShooterValid();
    }

    public void turnAndShoot(double powerShooter, double powerRotation, double powerScrews, double angle) {
        setShooterPower(powerShooter);
        pivotRotation(powerRotation, angle);
        pushRingUp(powerScrews);
        setShooterPower(0);
    }

    public void shootRing(double power){ //does the power vary?
        ringPusher.setPosition(0);
        ringPusher.setPosition(0);
        shooterMotor.setPower(power);
    }

    public void incrementTest(double power, double increment){
        screwMotor.setPower(power);
        if (opMode_iterative.gamepad1.a){
            power += increment;
            screwMotor.setPower(power);
        }
    }
    public boolean isTransitionValid() {
        return transitionValid;
    }

    public void setTransitionValid(boolean transitionValid) {
        this.transitionValid = transitionValid;
    }
    public boolean isShooterValid() {
        return shooterValid;
    }

    public void setShooterValid(boolean shooterValid) {
        this.shooterValid = shooterValid;
    }


    //=====TeleOp Methods=========//
    /*when shooting:
    1) intake the ring
    2) travels up the screw thingi
    3) reaches the top
    4) pushed into and out the shooter
     */
    public void fullControls(double screwPower, double shooterPower, double rotationPower, double rotationMultiplier, double rampAngleIncrement, double rampAngleMultiplier){
        pusherAndGrabberControls();
        screwsControls(screwPower);
        shooterControls(shooterPower, rotationPower, rotationMultiplier);
        rampControls(rampAngleIncrement, rampAngleMultiplier);
    }

    public void pusherAndGrabberControls(){
        if (opMode_iterative.gamepad2.y)
            togglePusher();
        if (opMode_iterative.gamepad1.y)
            //wobble grabber controls
        if (opMode_iterative.gamepad2.dpad_down)
            return;
            //wobble grabber controls
    }

    public void screwsControls(double power){
        if (opMode_iterative.gamepad2.b)
            setScrewPower(-power);
        if (opMode_iterative.gamepad2.a)
            setScrewPower(power);
    }

    public void shooterControls(double shooterPower, double rotationPower, double rotationMultiplier){
        if (opMode_iterative.gamepad1.x)
            setShooterPower(shooterPower);
        else if (opMode_iterative.gamepad2.x)
            setShooterPower(shooterPower);
        else if (opMode_iterative.gamepad2.right_bumper)
            setShooterPower(shooterPower);
        if (opMode_iterative.gamepad1.left_bumper)
            setRotationPower(-shooterPower); //turns shooter left
        if (opMode_iterative.gamepad1.right_bumper)
            setRotationPower(rotationPower); //turns shooter right
        if (opMode_iterative.gamepad2.left_stick_x != 0)
            setRotationPower(opMode_iterative.gamepad2.left_stick_x * rotationMultiplier); //move shooter left
    }

    public void rampControls(double rampAngleIncrement, double rampAngleMultiplier){
        if (opMode_iterative.gamepad1.b)
            rampAngleTeleOP += rampAngleIncrement;
        if (opMode_iterative.gamepad1.a)
            rampAngleTeleOP -= rampAngleIncrement;
        else if (opMode_iterative.gamepad2.right_stick_y != 0)
            rampAngleTeleOP += opMode_iterative.gamepad2.right_stick_y * rampAngleMultiplier;
        angleChanger.setPosition(rampAngleTeleOP);
    }

}