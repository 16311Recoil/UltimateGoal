package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class IntakeTestClass {
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private DcMotor intakeMotor;
    Sensors sensors;

    public IntakeTestClass (LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.addLine("Intake test Init Started");
        opMode.telemetry.update();

        intakeMotor = this.opMode.hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        opMode.telemetry.addLine("Intake test Init Completed");
        opMode.telemetry.update();

        sensors = new Sensors(this.opMode, true);


    }

    public IntakeTestClass(OpMode opMode) {
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Intake test Init Started");
        opMode_iterative.telemetry.update();

        intakeMotor = this.opMode.hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        opMode.telemetry.addLine("Intake test Init Completed");
        opMode.telemetry.update();

        sensors = new Sensors(this.opMode, false);
    }

    public void intakeTime(double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }

    public void intakeRing(double power){
        while (sensors.getTransitionValid()) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }

    public void test(double power){
        double powerIncrement = power;
        intakeMotor.setPower(powerIncrement);
        if (gamepad.a){
            powerIncrement += 0.05;
            intakeMotor.setPower(powerIncrement);
        }

    }

}
