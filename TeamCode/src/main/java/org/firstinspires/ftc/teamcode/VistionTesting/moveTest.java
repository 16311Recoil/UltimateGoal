package org.firstinspires.ftc.teamcode.VistionTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.Sensors;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.Drivetrain;

@TeleOp(name="Move Test", group="Iterative Opmode")
public class moveTest extends OpMode {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private Drivetrain dt;

        @Override
        public void init() {
        telemetry.addData("Status", "Initialized");

            try {

                //sensors = new Sensors();
                dt = new Drivetrain(this,null);
            } catch (InterruptedException e) {
                telemetry.addData("Status", "FAILED INIT");
            }

        telemetry.addData("Status", "Initialized");
    }

        @Override
        public void init_loop() {
    }
        @Override
        public void start() {
        runtime.reset();
    }

        @Override
        public void loop() {
            dt.moveTelop(gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x);
    }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
    }

    }










