package org.firstinspires.ftc.teamcode.NopeRopeLibs;


import android.graphics.Path;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.List;

public class Sensors {

    // Encoders used for odometry
    private Encoder front, left, right, rotation;
    private List<LynxModule> allHubs;
    private OpMode opmode;


    private static int count = 0;

    // Used for logging cycle reads.
    private ElapsedTime readTimer;

    private Rev2mDistanceSensor transition;
    private Rev2mDistanceSensor shooter;

    private DistanceSensor transitionValidation;
    private DistanceSensor shooterValidation;

    public Encoder[] encoders;

    public double[] positionVals;
    public double[] velocityVals;

    private enum EncoderIndexes{
        FRONT(0),
        LEFT(1),
        RIGHT(2),
        ROTATION(3);

        private int index;

        EncoderIndexes(int index){
            this.index = index;
        }

        public int getIndex() {
            return index;
        }
    }


    private boolean autoBulkRead = false;

    private static final String vuforiaKey = "";
    public static VuforiaLocalizer vuforia;
    private static final double SHOOTER_DIAMETER = 1.0;
    private static final double TRANSITION_LENGTH = 1.0;

    public Sensors(OpMode opMode, boolean LinearOpMode){

        this.opmode = opMode;
        // Gets all REV Hubs
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        transitionValidation = opMode.hardwareMap.get(DistanceSensor.class, "transitionDS");
        shooterValidation = opMode.hardwareMap.get(DistanceSensor.class, "shooterDS");

        transition = (Rev2mDistanceSensor) transitionValidation;
        shooter = (Rev2mDistanceSensor) shooterValidation;

        // Hardware Maps All of the Encoders
        front = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "front"));
        left = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "left"));
        right = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "right"));
        rotation = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rotation"));


        encoders = new Encoder[]{front, left, right, rotation};
        positionVals = new double[4];
        velocityVals = new double[4];

        //Set br to auto for now.
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Vuforia Initialization
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "WC");
        parameters.vuforiaLicenseKey = vuforiaKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4); // change?
    }


    // Bulk reads within the control loop
    public void autoBulkRead() throws AutoBulkReadNotSetException {
        if (autoBulkRead) {
           updateValues(encoders, positionVals, velocityVals);
        } else {
            throw new AutoBulkReadNotSetException("::Auto Not Enabled");
        }
    }

    public Runnable manualBulkRead(boolean loop){
            return () -> {
            for (LynxModule module : allHubs)
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            while(loop){
                updateValues(encoders, positionVals, velocityVals);
            }

        };

    }

    public void turnOnAutoBulkReads(){
        autoBulkRead = true;
        for (LynxModule module : allHubs)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
    public void turnOffAutoBulkReads(){
        autoBulkRead = false;
        for (LynxModule module : allHubs)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public boolean getShooterValid(){
        if (shooter.getDistance(DistanceUnit.INCH) <= SHOOTER_DIAMETER)
            return true;
        return false;
    }

    // color sensor
    public boolean getTransitionValid(){
        if (transition.getDistance(DistanceUnit.INCH) <= TRANSITION_LENGTH && count < 3) {
            count++;
            return true;
        }
        else
            return false;
    }

    //returns shooter angle
    public double getRotationAngle(){
        double currentVal = rotation.getCurrentPosition();

        return 0.0;


    }


    public Encoder[] getEncoders() {
        return encoders;
    }

    public void setEncoders(Encoder[] drivetrainEncoders) {
        this.encoders = drivetrainEncoders;
    }

    private void updateValues(Encoder[] encoders, double[] positionVals, double[] velocityVals){
        for(int i = 0; i < encoders.length; i++){
            positionVals[i] = encoders[i].getCurrentPosition();
            velocityVals[i] = encoders[i].getCorrectedVelocity();
        }

    }

    public double[] getPositionVals() {
        return positionVals;
    }

    public void setPositionVals(double[] positionVals) {
        this.positionVals = positionVals;
    }

    public double[] getVelocityVals() {
        return velocityVals;
    }

    public void setVelocityVals(double[] velocityVals) {
        this.velocityVals = velocityVals;
    }


    private class AutoBulkReadNotSetException extends Exception {
        AutoBulkReadNotSetException(String s){
            super(s);
        }

    }
}
