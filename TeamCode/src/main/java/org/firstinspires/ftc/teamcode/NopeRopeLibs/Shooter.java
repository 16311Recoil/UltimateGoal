package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private DcMotor intakeMotor;
    private DcMotor rotationMotor;
    private Servo outtakeServo;
    private final double TIME_FOR_INTAKE = 0;
    private final double flywheelPower = 0;
    private double angle = 0;



    public Shooter(LinearOpMode opMode)
    {
        this.opMode = opMode;
        opMode.telemetry.addLine("Shooter Init Started");
        opMode.telemetry.update();
        intakeMotor = this.opMode.hardwareMap.dcMotor.get("intakeMotor");
        rotationMotor = this.opMode.hardwareMap.dcMotor.get("rotationMotor");
        outtakeServo = this.opMode.hardwareMap.servo.get("outtakeServo");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        rotationMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setPower(0);
        rotationMotor.setPower(0);


        opMode.telemetry.addLine("Shooter Init Completed");
        opMode.telemetry.update();

    }
    public Shooter(OpMode opMode)
    {
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Shooter Init Started");
        opMode_iterative.telemetry.update();
        intakeMotor = this.opMode_iterative.hardwareMap.dcMotor.get("intakeMotor");
        rotationMotor = this.opMode_iterative.hardwareMap.dcMotor.get("rotationMotor");
        outtakeServo = this.opMode_iterative.hardwareMap.servo.get("rotationMotor");


        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotationMotor.setPower(0);
        intakeMotor.setPower(0);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        rotationMotor.setDirection(DcMotor.Direction.FORWARD);


        opMode_iterative.telemetry.addLine("Shooter Init Completed");
        opMode_iterative.telemetry.update();

    }




    public void setIntakePower(double power)
    {
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


    public void setAngle(double angle)
    {
    }

    public void pivotShooter(double power)
    {
        rotationMotor.setPower(power);
    }

    public void shoot(double power)
    {
    }









}
