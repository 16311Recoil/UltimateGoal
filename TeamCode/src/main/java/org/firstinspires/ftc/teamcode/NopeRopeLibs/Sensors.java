package org.firstinspires.ftc.teamcode.NopeRopeLibs;


import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.List;

public class Sensors {

    private BNO055IMU imu;
    // Encoders used for odometry
    private TrackingWheelLocalizer localizer;
    private List<LynxModule> allHubs;
    private OpMode opmode;
    private LinearOpMode opMode;


    private static int count = 0;

    // Used for logging cycle reads.
    private ElapsedTime readTimer;

    private Rev2mDistanceSensor transition;
    private Rev2mDistanceSensor shooter;

    private DistanceSensor transitionValidation;
    private DistanceSensor shooterValidation;

    public List<Encoder> encoders;

    public double[] positionVals;
    public double[] velocityVals;

    private boolean autoBulkRead = true;

    private static final String vuforiaKey = "";
    public static VuforiaLocalizer vuforia;
    private static final double SHOOTER_DIAMETER = 1.0;
    private static final double TRANSITION_LENGTH = 1.0;


    public Sensors(OpMode opMode){

        this.opmode = opMode;
        // Gets all REV Hubs
        LynxModuleUtil.ensureMinimumFirmwareVersion(opMode.hardwareMap);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        transitionValidation = opMode.hardwareMap.get(DistanceSensor.class, "transitionDS");
        shooterValidation = opMode.hardwareMap.get(DistanceSensor.class, "shooterDS");

        transition = (Rev2mDistanceSensor) transitionValidation;
        shooter = (Rev2mDistanceSensor) shooterValidation;



        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Hardware Maps All of the Encoders
       localizer = new TrackingWheelLocalizer(opMode.hardwareMap);
       //setEncoders(localizer.getEncoders());
        //rotation = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rotation"));
        //Set br to auto for now.

        // Vuforia Initialization
        /*
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "WC");
        parameters.vuforiaLicenseKey = vuforiaKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4); // change?

         */
    }

    public Sensors(LinearOpMode opMode){

        this.opmode = opMode;
        // Gets all REV Hubs
        LynxModuleUtil.ensureMinimumFirmwareVersion(opMode.hardwareMap);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        transitionValidation = opMode.hardwareMap.get(DistanceSensor.class, "transitionDS");
        shooterValidation = opMode.hardwareMap.get(DistanceSensor.class, "shooterDS");

        transition = (Rev2mDistanceSensor) transitionValidation;
        shooter = (Rev2mDistanceSensor) shooterValidation;

        // Hardware Maps All of the Encoders

        //rotation = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rotation"));
        //Set br to auto for now.

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        localizer = new TrackingWheelLocalizer(opMode.hardwareMap);
        setEncoders((List<Encoder>) localizer.getEncoders());

        // Vuforia Initialization
        /*
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "WC");
        parameters.vuforiaLicenseKey = vuforiaKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4); // change?

         */
    }
/*
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
*/
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
/*
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

 */

    //returns shooter angle
    //TODO
    public double getRotationAngle(){
        //double currentVal = rotation.getCurrentPosition();

        return 0.0;


    }


    public List<Encoder> getEncoders() {
        return encoders;
    }

    public void setEncoders(List<Encoder> encoders) {
        this.encoders = encoders;
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

    public boolean getPark(){
        return true;
    }

    public double getRawHeading(){
        return imu.getAngularOrientation().firstAngle;
    }
    public TrackingWheelLocalizer getLocalizer(){
        return localizer;
    }
    public List<Double> getWheelPos(){
        return localizer.getWheelPositions();
    }

    private class AutoBulkReadNotSetException extends Exception {
        AutoBulkReadNotSetException(String s){
            super(s);
        }

    }
}
