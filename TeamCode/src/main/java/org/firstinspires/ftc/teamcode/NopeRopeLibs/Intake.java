package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class Intake {
    private LinearOpMode auto; // auto -> sequential steps -> end on its own
    private OpMode teleop; // looping -> constantly running -> tell it to end

    private DcMotor intakeMotor;
    Sensors sensors;

    // Auto Constructor
    public Intake(LinearOpMode opMode, Sensors sensors) {
        this.auto = opMode;
        opMode.telemetry.addLine("Intake Init Started");
        opMode.telemetry.update();

        intakeMotor = this.auto.hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //* Changing the MODE -> velocity PID

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.sensors = sensors;

        opMode.telemetry.addLine("Intake test Init Completed");
        opMode.telemetry.update();
    }

    // TeleOp Constructor
    public Intake(OpMode opMode, Sensors sensors) {
        this.teleop = opMode;

        teleop.telemetry.addLine("Intake Init Started");
        teleop.telemetry.update();

        intakeMotor = this.teleop.hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.sensors = sensors;

        opMode.telemetry.addLine("Intake test Init Completed");
        opMode.telemetry.update();
    }


    public void intakeTime(double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }

    public void intakeRing(double power) {
        while (!sensors.getTransitionValid()) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }

    // Debounce? How do we fix debounce?
    // We are looking for a change from F -> T
    // T -> F
    // T -> F -> T
    public void incrementTest(double power, double increment) {
        intakeMotor.setPower(power);
        if (teleop.gamepad1.a) {
            power += increment;
            intakeMotor.setPower(power);
        }
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void turnOff() {
        intakeMotor.setPower(0);
    }

    /*
       Questions to consider
            - If i put this in a loop, what does the driver have to do to intake?
                Assume you fix debounce
                - Hold down Key (1), Count (2), Release Key(3)
            - Functionalities the driver needs?

     */
    public void moveTeleop(double power){
        if (teleop.gamepad2.b)// debounce issue
            intakeMotor.setPower(power);
        turnOff();
    }

}