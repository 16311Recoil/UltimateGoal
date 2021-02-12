package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private Servo wobblePivot;
    private Servo wobbleGrabber;


    private boolean transitionValid;


    private boolean shooterValid;

    private final double SERVO_POSITION_TO_ANGLE_FACTOR = 0; //Test for later
    private final double PUSH_OUT = 0.38;
    private final double PUSH_IN = 0.15;

    private double i = 0;
    //Aditiya likes penis and balls in his ass
    private final double WOBBLE_OUT = 0.55;
    private final double WOBBLE_IN = 1;
    private final double WOBBLE_CAPTURE = 0.6;
    private final double WOBBLE_RELEASE = 1;

    private boolean push = true;
    private boolean toggleWobbleP = true;
    private boolean toggleWobbleG = true;


    private double rampAngleTeleOP = 0;
    private boolean changeDpadUp = false;
    private boolean changeDpadDown = false;
    private double screwPower = 0.75;
    private boolean changeB2 = false;
    private boolean toggleRingPusher = false;
    private boolean changeB = false;
    private boolean changeX = false;
    private boolean changeX2 = false;
    private boolean changeY = false;
    private boolean changeY2 = false;
    private double servoPosition;
    private static final double revolution = 753.2;
    ElapsedTime pushTimer = new ElapsedTime();


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
        wobblePivot = this.opMode.hardwareMap.servo.get("wobblePivot");
        wobbleGrabber = this.opMode.hardwareMap.servo.get("wobbleGrabber");
        wobblePivot.setDirection(Servo.Direction.FORWARD);


        //angleChanger = this.opMode.hardwareMap.servo.get("angleChanger");



        rotationMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        screwMotor.setDirection(DcMotor.Direction.FORWARD);

        servoPosition = ringPusher.getPosition();


//        ringPusher.setDirection(Servo.Direction.FORWARD);
       // angleChanger.setDirection(Servo.Direction.FORWARD);

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
        shooterMotor = this.opMode_iterative.hardwareMap.dcMotor.get("shooterMotor");
        rotationMotor = this.opMode_iterative.hardwareMap.dcMotor.get("rotationMotor");
        screwMotor = this.opMode_iterative.hardwareMap.dcMotor.get("screwMotor");



        //Servos
        ringPusher = this.opMode_iterative.hardwareMap.servo.get("ringPusher");
        wobblePivot = this.opMode_iterative.hardwareMap.servo.get("wobblePivot");
        wobbleGrabber = this.opMode_iterative.hardwareMap.servo.get("wobbleGrabber");
        wobblePivot.setDirection(Servo.Direction.FORWARD);
        //angleChanger = this.opMode.hardwareMap.servo.get("angleChanger");



        rotationMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        screwMotor.setDirection(DcMotor.Direction.FORWARD);

        //ringPusher.setDirection(Servo.Direction.FORWARD);
        //angleChanger.setDirection(Servo.Direction.FORWARD);
        //screwMotor.setTargetPosition();

        ringPusher.setPosition(PUSH_IN);

        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        screwMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        opMode_iterative.telemetry.addLine("Shooter Init Completed");
        opMode_iterative.telemetry.update();

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

/*
    public void setShooterAngle(double angle){
        angleChanger.setPosition(angle * SERVO_POSITION_TO_ANGLE_FACTOR);
    }
*/
    public void togglePusher(boolean out){
        if (out){
            ringPusher.setPosition(PUSH_OUT);
        }
        else {
            ringPusher.setPosition(PUSH_IN);
        }
        push = !push;
    }

    public void toggleWobblePivot(boolean out){
        if (out){
            wobblePivot.setPosition(WOBBLE_OUT);
        }
        else {
            wobblePivot.setPosition(WOBBLE_IN);
        }
        toggleWobbleP = !toggleWobbleP;
    }

    public void toggleWobbleGrabber(boolean out){
        if (out){
            wobbleGrabber.setPosition(WOBBLE_CAPTURE);
        }
        else {
            wobbleGrabber.setPosition(WOBBLE_RELEASE);
        }
        toggleWobbleG = !toggleWobbleG;
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
    public void fullControls(double shooterPower, double rotationPower, double rotationMultiplier, double screwPower,double rampAngleIncrement, double rampAngleMultiplier){
        pusherAndGrabberControls();
        screwsControls(screwPower);
        shooterControls(shooterPower, rotationPower, rotationMultiplier);
        //rampControls(rampAngleIncrement, rampAngleMultiplier);
    }

    //public void pusherAndGrabberControls(){
      //  servoPosition = ringPusher.getPosition();
       // if (opMode_iterative.gamepad2.b /*&& changeB2*/ && withinPercent(servoPosition, PUSH_IN, 0.10))
          //  togglePusher(true);

        //else if (opMode_iterative.gamepad2.a /*&& changeB2*/ && withinPercent(servoPosition, PUSH_OUT, 0.10))
            //togglePusher(false);
        //changeB2 = opMode_iterative.gamepad2.b;
        /*
        if (opMode_iterative.gamepad1.y)
            //wobble grabber controls
        if (opMode_iterative.gamepad2.dpad_down)
            return;
            //wobble grabber controls

         */
    //}

    public void pusherAndGrabberControls(){


        if ((opMode_iterative.gamepad1.b && !changeB) || (opMode_iterative.gamepad2.b && !changeB2)){
            togglePusher(push);
            if (!push){
                pushTimer.reset();
            }
        }
        if (pushTimer.seconds() >= 1) {
            push = false;
            togglePusher(push);
        }
        changeB = opMode_iterative.gamepad1.b;
        changeB2 = opMode_iterative.gamepad2.b;

        if ((opMode_iterative.gamepad1.x && !changeX) || (opMode_iterative.gamepad2.x && !changeX2)){
           toggleWobbleGrabber(toggleWobbleG);
        }
        changeX = opMode_iterative.gamepad1.x;
        changeX2 = opMode_iterative.gamepad2.x;

        if ((opMode_iterative.gamepad1.y && !changeY) || (opMode_iterative.gamepad2.y && !changeY2)){
            toggleWobblePivot(toggleWobbleP);
        }
        changeY = opMode_iterative.gamepad1.y;
        changeY2 = opMode_iterative.gamepad2.y;

    }

    public boolean withinPercent(double compare, double threshold, double percent){
        return (compare >= (1 - percent) * threshold && compare <= threshold * (1 + percent));
    }

    public void screwsControls(double screwPower){
        //double currPos = screwMotor.getCurrentPosition();
        /*
        if(opMode_iterative.gamepad1.dpad_up && !changeDpadUp){
            screwPower += 0.1;
            if (screwPower > 1)
                screwPower = 1;
        }
        else if (opMode_iterative.gamepad1.dpad_down && !changeDpadDown && screwPower != -1){
            screwPower -= 0.1;
            if (screwPower < -1)
                screwPower = -1;
        }

         */
        /*
        if (opMode_iterative.gamepad2.left_bumper) {
            setScrewPower(-screwPower);
        }
        else if (opMode_iterative.gamepad2.right_bumper) {
            setScrewPower(screwPower);
        }
        else if (opMode_iterative.gamepad1.left_bumper) {
            setScrewPower(-screwPower);
        }
        else if (opMode_iterative.gamepad1.right_bumper) {
            setScrewPower(screwPower);
        }
        else
            setScrewPower(0);
       opMode_iterative.telemetry.addData("Screw Power", screwPower);
        changeDpadUp = opMode_iterative.gamepad1.dpad_up;
        changeDpadDown = opMode_iterative.gamepad1.dpad_down;

         */


    }

    private double pidController(boolean condition, double kp, double ki, double kd, double dt, double error, double lastError){
        double p = 0 , d = 0;
        if (condition){
            i += 0.5 * (error + lastError) * (dt);
            p = error;
            d = (error - lastError) / dt;
        }
        return kp * p + ki * i + kd * d;
    }



    public void shooterControls(double shooterPower, double rotationPower, double rotationMultiplier){
        if (opMode_iterative.gamepad1.right_trigger > 0.1)
            setShooterPower(shooterPower);
        else if (opMode_iterative.gamepad2.a)
            setShooterPower(shooterPower);
        //else if (opMode_iterative.gamepad1.y)
            //setShooterPower(0);
        else
           setShooterPower(0);
        if (opMode_iterative.gamepad1.left_bumper)
            setRotationPower(-rotationPower); //turns shooter left
        else if (opMode_iterative.gamepad1.right_bumper)
            setRotationPower(rotationPower); //turns shooter right
        else if (opMode_iterative.gamepad2.left_stick_x != 0)
            setRotationPower(opMode_iterative.gamepad2.left_stick_x * rotationMultiplier); //move shooter left
    }

    public void rampControls(double rampAngleIncrement, double rampAngleMultiplier){
        if (opMode_iterative.gamepad1.b)
            rampAngleTeleOP += rampAngleIncrement;
        if (opMode_iterative.gamepad1.a)
            rampAngleTeleOP -= rampAngleIncrement;
        else if (opMode_iterative.gamepad2.right_stick_y != 0)
            rampAngleTeleOP += opMode_iterative.gamepad2.right_stick_y * rampAngleMultiplier;
        //angleChanger.setPosition(rampAngleTeleOP);
    }
    public DcMotor getScrewMotor() {
        return screwMotor;
    }

    public void setScrewMotor(DcMotor screwMotor) {
        this.screwMotor = screwMotor;
    }

    public void setOut(){
        ringPusher.setPosition(PUSH_OUT);
    }
    public void setIn(){
        ringPusher.setPosition(PUSH_IN);
    }

}