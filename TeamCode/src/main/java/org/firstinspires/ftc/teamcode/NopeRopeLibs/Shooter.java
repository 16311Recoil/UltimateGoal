package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private DcMotor il;
    private DcMotor ir;
    private final double TIME_FOR_INTAKE = 0;
    private final double flywheelPower = 0;
    private double angle = 0;



    public Shooter(LinearOpMode opMode)
    {
        this.opMode = opMode;
        opMode.telemetry.addLine("Shooter Init Started");
        opMode.telemetry.update();
        il = this.opMode.hardwareMap.dcMotor.get("il");
        ir = this.opMode.hardwareMap.dcMotor.get("ir");

        il.setDirection(DcMotor.Direction.FORWARD);
        ir.setDirection(DcMotor.Direction.FORWARD);

        il.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ir.setPower(0);
        il.setPower(0);


        opMode.telemetry.addLine("Shooter Init Completed");
        opMode.telemetry.update();

    }
    public Shooter(OpMode opMode)
    {
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Shooter Init Started");
        opMode_iterative.telemetry.update();
        il = this.opMode_iterative.hardwareMap.dcMotor.get("il");
        ir = this.opMode_iterative.hardwareMap.dcMotor.get("ir");

        il.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ir.setPower(0);
        il.setPower(0);

        il.setDirection(DcMotor.Direction.FORWARD);
        ir.setDirection(DcMotor.Direction.FORWARD);


        opMode_iterative.telemetry.addLine("Shooter Init Completed");
        opMode_iterative.telemetry.update();

    }

    public DcMotor getIl() {
        return il;
    }

    public void setIl(DcMotor il) {
        this.il = il;
    }

    public DcMotor getIr() {
        return ir;
    }

    public void setIr(DcMotor ir) {
        this.ir = ir;
    }



    public void setIntakePower(double power)
    {
        il.setPower(power);
        ir.setPower(power);

    }

    public void intakeTime(double power) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < TIME_FOR_INTAKE) {
            setIntakePower(power);
        }
        setIntakePower(0);
    }


    public void setAngle(double angle)
    {
    }







}
