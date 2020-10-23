package org.firstinspires.ftc.teamcode.NopeRopeLibs;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.List;

public class Sensors {

    // Encoders used for odometry
    private Encoder front, left, right;
    private List<LynxModule> allHubs;

    // Used for logging cycle reads.
    private ElapsedTime readTimer;


    public Encoder[] drivetrainEncoders;

    public double[] positionVals;
    public double[] velocityVals;

    private final int FRONT = 0;
    private final int LEFT = 1;
    private  final int RIGHT = 2;

    private boolean autoBulkRead = false;

    // WC replacement --- think about it
    // Transition Sensor???
    //

    // Webcam
    private final static String vuforiaKey = "";
    public static VuforiaLocalizer vuforia;

    public Sensors(OpMode opMode){
        // Gets all REV Hubs
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        // Hardware Maps All of the Encoders
        front = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "front"));
        left = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "left"));
        right = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "right"));

        drivetrainEncoders = new Encoder[]{front, left, right};

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
            front.getCurrentPosition();
            left.getCurrentPosition();
            right.getCurrentPosition();
        } else {
            throw new AutoBulkReadNotSetException("::Auto Not Enabled");
        }
    }

    public Runnable manualBulkRead(boolean loop){
            return () -> {
            for (LynxModule module : allHubs)
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            while(loop){
                front.getCurrentPosition();
                left.getCurrentPosition();
                right.getCurrentPosition();

                updateEncoders();
            }

        };

    }
    public void updateEncoders(){}
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

    public Encoder[] getDrivetrainEncoders() {
        return drivetrainEncoders;
    }

    public void setDrivetrainEncoders(Encoder[] drivetrainEncoders) {
        this.drivetrainEncoders = drivetrainEncoders;
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
