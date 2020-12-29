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


    public NopeRope() {
        //..../
    }

    public NopeRope(LinearOpMode opMode) { //decide which path to take here?
    }

    public NopeRope(OpMode opMode) {
    }


    public void teleOpControls() {

        // driver 1 controls the drivetrain
        drivetrain.moveTelop(-teleOp.gamepad1.left_stick_x, teleOp.gamepad1.right_stick_x, -teleOp.gamepad1.left_stick_x);
        intake.intakeControls(0);
        shooter.fullControls(0,0,0,0,0,0);
        // for shooter <- who controls the shooter? with what methods?
        //shooter.moveTeleop(null,null,null,null); //parameters?
        // don't worry about TeleOp vision right now.

    }

    /* METHODS:A
            - intake and shoot in auto (intake, shooter & sensors)
            -  anything in auto??
     */

    public void intakeAndShoot ( double power, double powerShooter, double powerRotation, double powerScrews, double angle){
        intake.intakeRing(power);
        shooter.turnAndShoot(powerShooter, powerRotation, powerScrews, angle);
    }

    public void pickUpWobbleGoal () {
    }
    public void dropWobbleGoal () {
    }

/*
    Auto path based on starter stack configuration
        - A: no discs (lower left)
        - B: one discs (right middle)
        - C: four discs (upper left)
 */

    //is there a specific power to set to, since there's no value coming in
    public void posA ( double distance, double power, double timeout){
        //distance = (-48,0)
        drivetrain.moveForward(distance, power, timeout); //then drop wobble goal
        //strafe right to (-12,0)
        shooter.turnAndShoot();
        //move back to (-12,-48), then strafe right to (-24,-48)
        pickUpWobbleGoal();
        //strafe right to (-48,-48), then move forward to (-48,0)
        dropWobbleGoal();
        //move forward to launch line (-48,12)
    }

    public void posB ( double distance, double power, double timeout){
        //distance = (-48,24)
        drivetrain.moveForward(distance, power, timeout);
        dropWobbleGoal();
        //move back to (-48,0), then strafe right to (-12,0)
        shooter.turnAndShoot();
        //move back to (-12,-48), then strafe right to (-24,-48)
        pickUpWobbleGoal();
        //strafe right to (-48,-48), then move forward to (-48,24)
        dropWobbleGoal();
        //move back to launch line (-48,12)

    }

    public void posC ( double distance, double power, double timeout)
    { //distance is the end target
        //distance = (-48,48)
        drivetrain.moveForward(distance, power, timeout);
        dropWobbleGoal();
        //move back to (-48,0), then strafe right to (-12,0)
        shooter.turnAndShoot();
    /*
        one: wobble goal & park
            - move back to (-12,-48), then strafe right to (-24,-48), collect wobble goal
            - strafe right to (-48,-48), then move forward to (-48,48), drop wobble goal
            - move back to launch line (-48,12)
     */
    /*
        two: use starter stack & park
            - strafe right to (-36,0), then move back to (-36,-24)
            - collect rings
            - move forward to (-36,0), shoot rings
            - move forward to (-36,12)
     */
    /*
        three: use starter stack, wobble goal & park
            - strafe right to (-36,0), then move back to (-36,-24), collect rings
            - move forward to (-36,0), shoot rings
            - move back to (-36,-48), collect wobble goal
            - move forward to (-36,48), strafe right to (-48,48), drop wobble goal
            - move back to (-48,12)
     */
    }

    public void parkRobot () {
        while (sensors.) //to detect the launch line
            drivetrain.setAllMotors(0);
    }

    public void wobblegoalControls () {
        if (teleOp.gamepad1.dpad_down)
            dropWobbleGoal();
        if (teleOp.gamepad1.dpad_up)
            pickUpWobbleGoal();
        if (teleOp.gamepad1.y)
        //toggles wobble grabber
    }

}
