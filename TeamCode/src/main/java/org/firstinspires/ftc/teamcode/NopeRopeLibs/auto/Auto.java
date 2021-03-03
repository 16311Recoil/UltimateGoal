package org.firstinspires.ftc.teamcode.NopeRopeLibs.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.NopeRope;

public class Auto extends LinearOpMode {
    NopeRope robot;
    AutoBuilder auto;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NopeRope(this);
        auto = new AutoBuilder();
        int initSize = auto.getCurrent().size();

        waitForStart();

        /*



        for(int i = 0; i < auto.getCurrent().size(); i++){
            if(i == (initSize - 2)){
                robot.scan();
            }
            robot.performMovement(auto.getCurrent().get(i));

            if()
        }


         */


    }
}
