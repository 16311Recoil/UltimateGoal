package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class NopeRope {

    private OpMode teleOp;
    private LinearOpMode auto;

    private Drivetrain drivetrain;
    private Sensors sensors;
    private Shooter shooter;
    private Webcam webcam;
    private Intake intake;
    private VisionTensorFlow vision1;


    public NopeRope(){
        //..../
    }

    public NopeRope(LinearOpMode opMode){
    }

    public NopeRope(OpMode opMode){
    }

    public void teleOpControls(){
        // driver 1 controls the drivetrain
        drivetrain.moveTelop(teleOp.gamepad1.left_stick_x, teleOp.gamepad1.left_stick_y, teleOp.gamepad1.right_stick_x);
        // for shooter <- who controls the shooter? with what methods?
        //shooter.moveTeleop(null,null,null,null);//parameters?
        // don't worry about TeleOp vision right now.

    }



}
