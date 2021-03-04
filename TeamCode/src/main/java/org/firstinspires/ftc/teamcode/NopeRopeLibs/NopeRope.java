package org.firstinspires.ftc.teamcode.NopeRopeLibs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NopeRopeLibs.auto.MovementConstants;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.Drivetrain;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.vision.VisionTensorFlow;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.vision.Webcam;

public class NopeRope {

    private OpMode teleOp;
    private LinearOpMode auto;

    private static final int SHOOTER = 0;
    private static final int TRANSITION = 1;
    private static final int INTAKE = 2;

    private static double MULTIPLIER = 0.787401574803;

    private final Drivetrain drivetrain;
    private final Sensors sensors;
    private final Shooter shooter;
    private Webcam webcam;
    private Intake intake;
    private VisionTensorFlow vision1;
    private boolean[] sensorsArray;
    private FtcDashboard dashboard;



    public NopeRope(LinearOpMode opMode) throws InterruptedException { //decide which path to take here?
        auto = opMode;
        sensors = new Sensors(auto, dashboard);
        drivetrain = new Drivetrain(auto, sensors.getLocalizer());
        shooter = new Shooter(auto);
        vision1 = new VisionTensorFlow();
        sensorsArray = new boolean[2];
       // sensorsArray[0] = sensors.getShooterValid();
       // sensorsArray[1] = sensors.getTransitionValid();

        // Webcam in sensors?
    }

    public NopeRope(OpMode opMode) throws InterruptedException {
        teleOp = opMode;
        sensors = new Sensors(teleOp);
        drivetrain = new Drivetrain(teleOp, sensors.getLocalizer());
        shooter = new Shooter(teleOp);
        intake = new Intake(teleOp);
        sensorsArray = new boolean[2];
//        sensorsArray[0] = sensors.getShooterValid();
 //       sensorsArray[1] = sensors.getTransitionValid();

        //updateDrivetrainAngle();

    }

    public void updateDrivetrainAngle(){
        drivetrain.setExternalHeading(sensors.getRawExternalHeading());
    }


    public void teleOpControls() {
        drivetrain.moveTelop(-teleOp.gamepad1.right_stick_x * MULTIPLIER ,teleOp.gamepad1.right_stick_y * MULTIPLIER,  teleOp.gamepad1.left_stick_x);
        intake.intakeControls(1); // test power
        shooter.fullControls(0.9,0,0,0.75,0,0);
        shooter.setRevBoolean(sensors.isScrewRevolution());

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
        //shooter.turnAndShoot();
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
        //shooter.turnAndShoot(0,0,0,0);
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
        //shooter.turnAndShoot();
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
        while (!sensors.getPark()) //to detect the launch line
            drivetrain.setAllMotors(0);
    }

    public void revolutionValid(){
        shooter.setRevolutionValid(sensors.isScrewRevolution());
    }

    /*
    public void updateTransition(){
        boolean newTransitionVal = sensors.getTransitionValid();

        if (isChange(newTransitionVal, sensorsArray[TRANSITION]))
            intake.setTransitionValid(newTransitionVal);

        sensorsArray[TRANSITION] = newTransitionVal;

    }
    public void updateShooter(){

        boolean newShooterVal = sensors.getShooterValid();
        boolean newTransitionVal = sensors.getTransitionValid();


        if (isChange(newShooterVal, sensorsArray[SHOOTER]))
            shooter.setShooterValid(newShooterVal);
        if (isChange(newTransitionVal, sensorsArray[TRANSITION]))
            shooter.setTransitionValid(newTransitionVal);

        sensorsArray[SHOOTER] = newShooterVal;
        sensorsArray[TRANSITION] = newTransitionVal;
    }
*/
    private boolean isChange(boolean a, boolean b){
        return a ^ b;
    }

    // TODO
    public void wobblegoalControls () {
        if (teleOp.gamepad1.dpad_down)
            dropWobbleGoal();
        if (teleOp.gamepad1.dpad_up)
            pickUpWobbleGoal();
        if (teleOp.gamepad1.y)
            return;
        //toggles wobble grabber
    }
    public Sensors getSensors() {
        return sensors;
    }
    public Drivetrain getDrivetrain(){
        return drivetrain;
    }
    public Shooter getShooter(){
        return shooter;
    }
    public Intake getIntake(){
        return intake;
    }


    public void performMovement(MovementConstants c) {
        this.drivetrain.moveToPositionPID(c.xTarget, c.yTarget, c.zTarget, c.timeout, c.constants);
    }
}
