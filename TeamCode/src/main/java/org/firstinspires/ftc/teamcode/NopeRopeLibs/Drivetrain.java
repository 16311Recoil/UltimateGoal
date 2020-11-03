package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.EscapeAnalysis;

import java.util.Arrays;
import java.util.Map;

public class Drivetrain {

    public LinearOpMode opMode;
    public OpMode opMode_iterative;
    private DcMotor fl, fr, bl, br;
    private double multiplier;
    private int multiCounter = 1;
    private double[] multipliers = {0.3, 0.55 , 1};
    private String[] multipliersTelemetry = {"LOW POWER", "REGULAR POWER", "HIGH POWER"};


    public Drivetrain(LinearOpMode opMode, ElapsedTime timer, Map<String, Double> sensorVals) throws InterruptedException {
        this.opMode = opMode;

        fl =  this.opMode.hardwareMap.dcMotor.get("fl");
        fr =  this.opMode.hardwareMap.dcMotor.get("fr");
        bl =  this.opMode.hardwareMap.dcMotor.get("bl");
        br =  this.opMode.hardwareMap.dcMotor.get("br");


        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opMode.telemetry.addLine("Drivetrain Init Completed");
        opMode.telemetry.update();
    }

    public Drivetrain(OpMode opMode, ElapsedTime timer, Map<String, Double> sensorVals) throws InterruptedException {
        this.opMode_iterative = opMode;

        opMode_iterative.telemetry.addLine("Drivetrain update");
        opMode_iterative.telemetry.update();



        fl =  this.opMode_iterative.hardwareMap.dcMotor.get("fl");
        fr =  this.opMode_iterative.hardwareMap.dcMotor.get("fr");
        bl =  this.opMode_iterative.hardwareMap.dcMotor.get("bl");
        br =  this.opMode_iterative.hardwareMap.dcMotor.get("br");

        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opMode_iterative.telemetry.addLine("Drivetrain Complete");
        opMode_iterative.telemetry.update();
    }

    /* ============================================== UTILITY METHODS ==============================================================*/

    public void setAllMotors(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void turn(double power, boolean turnRight){
        int turnMultiplier = -1;
        if (turnRight){
            turnMultiplier = 1;
        }
        fl.setPower(power*turnMultiplier);
        fr.setPower(-power*turnMultiplier);
        bl.setPower(power*turnMultiplier);
        br.setPower(-power*turnMultiplier);
    }

    public double getDistance(){
        return 0;
        //placeholder for odom method in sensors class - REMOVE AFTER REAL METHOD IMPLEMENTED
        //Ctrl F marker for methods that need to be updated
    }

    public double getAngle(){
        return 0;
        //placeholder for odom method in sensors class - REMOVE AFTER REAL METHOD IMPLEMENTED
        //Ctrl F marker for methods that need to be updated
    }

    public void moveForward(double distance, double power, double timeout){
        double currentPos = getDistance();
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < timeout && currentPos < distance){
            currentPos = getDistance();
            setAllMotors(power);
        }
        setAllMotors(0);
    }

    public void turnTo (double power, double desiredAngle, double timeout){
        ElapsedTime timer = new ElapsedTime();
        double currentAngle = getAngle(); //radians
        if (Math.abs(desiredAngle - currentAngle) > Math.PI) {
            desiredAngle += 360;
        }
        boolean turnRight = ((currentAngle-desiredAngle) < (desiredAngle-currentAngle));
        if (turnRight){
            turn(power, true);
            while ((currentAngle > desiredAngle) && (timer.seconds() < timeout)){ }
        }
        else {
            turn(power, false);
            while ((desiredAngle > currentAngle) && (timer.seconds() < timeout)){ }
        }
        setAllMotors(0);
    }

    //================================================================= Tele-Op Methods ===============================================================//

    public void moveTelop(double x, double y, double z) {
        double v_d = Math.hypot(x, y) * Math.signum(x) * Math.signum(y);
        double netTheta = Math.atan2(x, y);
        if (v_d < 0.05)
            v_d = 0;
        if (v_d > 0.95)
            v_d = 1;
        toggleSpeed();
        fl.setPower(multiplier * (v_d * (Math.sin((netTheta) + Math.PI / 4))) - z);
        fr.setPower(multiplier * (v_d * (Math.sin((netTheta) + Math.PI / 4))) + z);
        bl.setPower(multiplier * (v_d * (Math.cos((netTheta) + Math.PI / 4))) - z);
        br.setPower(multiplier * (v_d * (Math.cos((netTheta) + Math.PI / 4))) + z);
    }


    //Have to make sure to add these controls in the gamepad class
    public void toggleSpeed() {
        /*if ((opMode_iterative.gamepad1.dpad_down && !changeDpadDown) && multiCounter > 0) {
            multiCounter--;
        }
        else if ((opMode_iterative.gamepad1.dpad_up && !changeDpadUp) && multiCounter < 2) {
            multiCounter++;
        }*/
        multiplier = multipliers[multiCounter];
        opMode_iterative.telemetry.addLine(multipliersTelemetry[multiCounter]);

        //changeDpadDown = opMode_iterative.gamepad1.dpad_down;
        //changeDpadUp = opMode_iterative.gamepad1.dpad_up;
        opMode_iterative.telemetry.update();
    }

}
