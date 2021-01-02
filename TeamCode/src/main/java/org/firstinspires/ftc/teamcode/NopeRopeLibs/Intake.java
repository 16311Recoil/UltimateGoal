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
    private boolean transitionValid;

    // Auto Constructor
    public Intake(LinearOpMode opMode) {
        this.auto = opMode;
        opMode.telemetry.addLine("Intake Init Started");
        opMode.telemetry.update();

        intakeMotor = this.auto.hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        transitionValid = false;
        //* Changing the MODE -> velocity PID

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        opMode.telemetry.addLine("Intake test Init Completed");
        opMode.telemetry.update();
    }

    // TeleOp Constructor
    public Intake(OpMode opMode) {
        this.teleop = opMode;

        teleop.telemetry.addLine("Intake Init Started");
        teleop.telemetry.update();

        intakeMotor = this.teleop.hardwareMap.dcMotor.get("intakeMotor");
        transitionValid = false;

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


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
        while (getTransitionValid()) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }

    public void setTransitionValid(boolean valid){
        this.transitionValid = valid;
    }


    public boolean getTransitionValid() {
        return transitionValid;
    }


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

    public void intakeControls(double power){
        if (teleop.gamepad1.right_trigger > 0)
            intakeMotor.setPower(power);
        else if (teleop.gamepad2.right_trigger > 0)
            intakeMotor.setPower(power);
        if (teleop.gamepad1.left_trigger > 0)
            intakeMotor.setPower(-power);
        else if (teleop.gamepad2.left_trigger > 0)
            intakeMotor.setPower(-power);
    }

}