package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Shooter {
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private DcMotor shooterMotor; //the actual shooter
    private DcMotor rotationMotor; //larger black wheel
    private DcMotor screwMotor;
    private DcMotor intakeMotor;
    Servo ringPusher;
    Servo shooterAngle;

    private final double TIME_FOR_INTAKE = 0;
    private final double flywheelPower = 0;
    private double angle = 0;
    // Create a Servo Object



    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.addLine("Shooter Init Started");
        opMode.telemetry.update();
        //Motors
        shooterMotor = this.opMode.hardwareMap.dcMotor.get("shooterMotor");
        rotationMotor = this.opMode.hardwareMap.dcMotor.get("rotationMotor");
        screwMotor = this.opMode.hardwareMap.dcMotor.get("screwMotor");
        intakeMotor = this.opMode.hardwareMap.dcMotor.get("intakeMotor");
        //Servos
        ringPusher = this.opMode.hardwareMap.servo.get("ringPusher");
        shooterAngle = this.opMode.hardwareMap.servo.get("shooterAngle");

        // Instantiate a servo object.

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        rotationMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setPower(0);
        rotationMotor.setPower(0);
        screwMotor.setPower(0);
        intakeMotor.setPower(0);


        opMode.telemetry.addLine("Shooter Init Completed");
        opMode.telemetry.update();

    }

    public Shooter(OpMode opMode) {
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Shooter Init Started");
        opMode_iterative.telemetry.update();
        shooterMotor = this.opMode_iterative.hardwareMap.dcMotor.get("shooterMotor");
        rotationMotor = this.opMode_iterative.hardwareMap.dcMotor.get("rotationMotor");
        screwMotor = this.opMode_iterative.hardwareMap.dcMotor.get("screwMotor");
        intakeMotor = this.opMode_iterative.hardwareMap.dcMotor.get("intakeMotor");

        ringPusher = this.opMode_iterative.hardwareMap.servo.get("ringPusher");
        shooterAngle = this.opMode_iterative.hardwareMap.servo.get("shooterAngle");


        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setPower(0);
        rotationMotor.setPower(0);
        screwMotor.setPower(0);
        intakeMotor.setPower(0);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        rotationMotor.setDirection(DcMotor.Direction.FORWARD);


        opMode_iterative.telemetry.addLine("Shooter Init Completed");
        opMode_iterative.telemetry.update();

    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
        rotationMotor.setPower(power);

    }

    public void intakeTime(double power) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < TIME_FOR_INTAKE) {
            setIntakePower(power);
        }
        setIntakePower(0);
    }

    // OBJECTIVE: Given a target angle, increment the servo position, until that angle is met.
    // How are you going to get feedback from a servo angle?
    // Look into using a potentiometer (look into what sensors you can use).
    //  Is there a loop? Recursive If?


    //  How to get data from the sensors class?
    public double sensor() {
        Sensors angles = new Sensors();
        return //what methods from the sensors class?

    }

    // No end point?
    // if () {.setPower(0);}
    // .getCurrentPosition()
    public void pivotShooter(double power) { //?

        double target = sensor();
        double CurrentMotorPosition = rotationMotor.getCurrentPosition();
        while (CurrentMotorPosition < target) {
            rotationMotor.setPower(power);
            CurrentMotorPosition = rotationMotor.getCurrentPosition();
        }
        rotationMotor.setPower(0);

    }

    // OBJECTIVE: What needs to happen for the robot to shoot a disc?
    // 1.) The Disc Has to be present.
    // 2.) Something needs to move to push it into position.
    // 3.) There needs to be some control of the power of the shooter?
    // turnOnShooter()
    // pushRing()
    /*  turnAndShoot(){
    //      turnOnShooter();
    //      positionShooter();
    //      pushRing();
    //      turnOffShooter(); Or set Shooter Power to Zero

     */


    public void turnOnShooter(double power) { //does the power vary?
        shooterMotor.setPower(power);
    }


    public void turnOffShooter() {
        shooterMotor.setPower(0);
    }

    public void turnAndShoot() {
        turnOnShooter();
        pivotShooter();
        pushRingUp();
        turnOffShooter();

    }

    public void intakeRing(double power){
        while (!discPresent()) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);

    }

    public void pushRingUp(double power) {
        while discPosition(){
            screwMotor.setPower(power);
        }
        screwMotor.setPower(0);

    }

    public boolean discPresent() {
        //sensorthatsensesifdiscsthere
        return // true if there are already three rings

    }

    public boolean discPosition(){ //distance sensor
        return //true if disc is at the top
    }

    public void rotateWheel(double power){ //under what conditions? probably use a distance sensor?

        double currentAngle = sensor();
        double targetAngle = sensor();

        while (currentAngle < targetAngle){
            double error = targetAngle - currentAngle;
            rotationMotor.setPower(power);
            currentAngle = sensor();
        }


    }

    public void angleShooter(){

        double currentAngle = sensor();
        double targetAngle = sensor();

        while (currentAngle < targetAngle){
            double error = targetAngle - currentAngle;
            shooterAngle.setDirection(error);
            currentAngle = sensor();
        }

    }

    public void shootRing(double power){ //does the power vary?
        ringPusher.setPosition();
        ringPusher.setPosition();
        shooterMotor.setPower(power);

    }

}