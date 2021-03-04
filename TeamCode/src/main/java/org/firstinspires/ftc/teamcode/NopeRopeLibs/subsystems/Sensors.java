package org.firstinspires.ftc.teamcode.NopeRopeLibs.subsystems;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.motion.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.NopeRopeLibs.vision.RingDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class Sensors {

    public BNO055IMU gyro;
    private Orientation angles;
    // Encoders used for odometry
    private TwoWheelLocalizer localizer;
    private List<LynxModule> allHubs;
    private OpMode teleOp;
    private LinearOpMode auto;

    // Used for logging cycle reads.
    private ElapsedTime readTimer;

    private Rev2mDistanceSensor transition;
    private Rev2mDistanceSensor shooter;

    private DistanceSensor transitionValidation;
    private DistanceSensor shooterValidation;

    NormalizedColorSensor colorSensor;
    DcMotor motor;
    View relativeLayout;
    final float[] hsvValues = new float[3];
    float gain = 2;

    private FtcDashboard dashboard;

    public List<Encoder> encoders;

    public double[] positionVals;
    public double[] velocityVals;
    private static int stackSize;

    private OpenCvWebcam webcam;

    private boolean autoBulkRead = true;

    private static final String vuforiaKey = "";
    public static VuforiaLocalizer vuforia;
    private static final double SHOOTER_DIAMETER = 1.0;
    private static final double TRANSITION_LENGTH = 1.0;


    public Sensors(OpMode opMode){
        this.teleOp = opMode;
        // Gets all REV Hubs
        LynxModuleUtil.ensureMinimumFirmwareVersion(opMode.hardwareMap);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        transitionValidation = opMode.hardwareMap.get(DistanceSensor.class, "transitionDS");
        shooterValidation = opMode.hardwareMap.get(DistanceSensor.class, "shooterDS");

        transition = (Rev2mDistanceSensor) transitionValidation;
        shooter = (Rev2mDistanceSensor) shooterValidation;

        int relativeLayoutId = opMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", opMode.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) opMode.hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            colorSensor = teleOp.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(true);
            }

            // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }


        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Hardware Maps All of the Encoders
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        localizer = new TwoWheelLocalizer(opMode.hardwareMap, this);
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

    public Sensors(LinearOpMode opMode, FtcDashboard dashboard){
        this.auto = opMode;
        // Gets all REV Hubs
        LynxModuleUtil.ensureMinimumFirmwareVersion(auto.hardwareMap);
        allHubs = auto.hardwareMap.getAll(LynxModule.class);

        transitionValidation = auto.hardwareMap.get(DistanceSensor.class, "transitionDS");
        shooterValidation = auto.hardwareMap.get(DistanceSensor.class, "shooterDS");

        transition = (Rev2mDistanceSensor) transitionValidation;
        shooter = (Rev2mDistanceSensor) shooterValidation;

        stackSize = -1;
        int relativeLayoutId = auto.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", auto.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) auto.hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            colorSensor = auto.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(true);
            }

            // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }

        // Hardware Maps All of the Encoders

        //rotation = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rotation"));
        //Set br to auto for now.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = auto.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        localizer = new TwoWheelLocalizer(opMode.hardwareMap, this);
        setEncoders((List<Encoder>) localizer.getEncoders());


        this.dashboard = FtcDashboard.getInstance();

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

    private void updateGyro()
    {
        angles = gyro.getAngularOrientation();
    }

    public boolean isScrewRevolution(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return (colors.blue > colors.red);
    }

    // Bulk Reads Sensor Data to update all values for use.
    public void updateSensorVals(){
        updateGyro();

    }
    // Determine which is yaw, pitch, and roll.
    public double getRawExternalHeading()
    {
        updateGyro();
        return angles.firstAngle;
    }
    public double getSecondAngle()
    {
        updateGyro();
        return angles.secondAngle;
    }
    public double getThirdAngle()
    {
        //sensorRange.getDistance(DistanceUnit.MM))
        updateGyro();
        return angles.thirdAngle;
    }
    public TwoWheelLocalizer getLocalizer(){
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

    public void startScanning(){
        webcam.setPipeline(new RingDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }
    public void scan(){
        webcam.setPipeline(new RingDetectionPipeline());
        TelemetryPacket packet = new TelemetryPacket();
        stackSize = RingDetectionPipeline.stackSize;
        if (stackSize < 0) {
            packet.addLine("ERROR IN DETECTING STACK SIZE");
            auto.telemetry.addLine("ERROR IN DETECTING STACK SIZE");
            stackSize = 0;
        } else{
            packet.put("Stack Size", stackSize);
            auto.telemetry.addData("Stack Size", stackSize);
        }
        dashboard.sendTelemetryPacket(packet);
    }


    public void stopScanning(){
        webcam.stopStreaming();
    }

}