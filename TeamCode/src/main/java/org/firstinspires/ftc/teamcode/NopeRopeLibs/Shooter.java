package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Shooter {
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private DcMotor shooterMotor; //Outtake Wheels
    private DcMotor rotationMotor; //Larger Rotation Device
    private DcMotor screwMotor; //Motor to power the screws
    private DcMotor intakeMotor; //Intake
    Servo ringPusher; //Pushes the ring into the outtake
    Servo angleChanger; //Adjusts the angle of the ramp/shooter
    Sensors sensors;

    private final double SERVO_POSITION_TO_ANGLE_FACTOR = 0; //Test for later
    boolean push = true;


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
        angleChanger = this.opMode.hardwareMap.servo.get("angleChanger");



        rotationMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        screwMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        ringPusher.setDirection(Servo.Direction.FORWARD);
        angleChanger.setDirection(Servo.Direction.FORWARD);

        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        screwMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        opMode.telemetry.addLine("Shooter Init Completed");
        opMode.telemetry.update();

        sensors = new Sensors(this.opMode);
    }

    public Shooter(OpMode opMode) {
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Shooter Init Started");
        opMode_iterative.telemetry.update();

        //Motors
        shooterMotor = this.opMode.hardwareMap.dcMotor.get("shooterMotor");
        rotationMotor = this.opMode.hardwareMap.dcMotor.get("rotationMotor");
        screwMotor = this.opMode.hardwareMap.dcMotor.get("screwMotor");
        intakeMotor = this.opMode.hardwareMap.dcMotor.get("intakeMotor");

        //Servos
        ringPusher = this.opMode.hardwareMap.servo.get("ringPusher");
        angleChanger = this.opMode.hardwareMap.servo.get("angleChanger");



        rotationMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        screwMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        ringPusher.setDirection(Servo.Direction.FORWARD);
        angleChanger.setDirection(Servo.Direction.FORWARD);

        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        screwMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        opMode.telemetry.addLine("Shooter Init Completed");
        opMode.telemetry.update();

        sensors = new Sensors(this.opMode);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
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

    public void intakeTime(double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            setIntakePower(power);
        }
        setIntakePower(0);
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

    public void pivotRotation (double power, double desiredAngle){
        desiredAngle = Math.toRadians(desiredAngle);
        double currentAngle = sensors.getRotationAngle(); //radians
        if (Math.abs(desiredAngle - currentAngle) > Math.PI) {
            desiredAngle += Math.PI * 2;
        }
        boolean turnRight = ((currentAngle-desiredAngle) < (desiredAngle-currentAngle));
        if (turnRight){
            setRotationPower(power);
            while (currentAngle > desiredAngle){ }
        }
        else {
            setRotationPower(-power);
            while (desiredAngle > currentAngle){ }
        }
        setRotationPower(0);
    }

    public void setShooterAngle(double angle){
        angleChanger.setPosition(angle * SERVO_POSITION_TO_ANGLE_FACTOR);
    }

    public void togglePusher(){
        if (push){

        }
    }

    public void pushRingUp(double power) {
        while (!sensors.getShooterValid()){
            screwMotor.setPower(power);
        }
        screwMotor.setPower(0);
    }

    public void intakeRing(double power){
        while (sensors.getTransitionValid()) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }

    public boolean discsFull() {
        return sensors.getTransitionValid();
    }

    public boolean discAtTop() { //distance sensor
        return sensors.getShooterValid();
    }

    public void turnAndShoot(double powerShooter, double powerRotation, double powerScrews, double angle) {
        setShooterPower(powerShooter);
        pivotRotation(powerRotation, angle);
        pushRingUp(powerScrews);
        setShooterPower(0);
    }

    public void shootRing(double power){ //does the power vary?
        ringPusher.setPosition();
        ringPusher.setPosition();
        shooterMotor.setPower(power);
    }

}