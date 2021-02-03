package org.firstinspires.ftc.teamcode.NopeRopeLibs.motion;

import android.media.midi.MidiDevice;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 360;
    public static double WHEEL_RADIUS = 0.984252; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 1.5; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    private List<Encoder> encoders;
    private Encoder rightEncoder, leftEncoder, middleEncoder;

    public TrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "br"));
        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "shooterMotor"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bl"));




        encoders = new ArrayList<Encoder>();
        encoders.add(rightEncoder);
        encoders.add(leftEncoder);
        encoders.add(middleEncoder);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        //leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;

    }

    @NonNull

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(middleEncoder.getCurrentPosition()),
                encoderTicksToInches(leftEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                encoderTicksToInches(middleEncoder.getRawVelocity()),
                encoderTicksToInches(leftEncoder.getRawVelocity())
        );
    }
    public List<Encoder> getEncoders(){
        return encoders;
    }
}
