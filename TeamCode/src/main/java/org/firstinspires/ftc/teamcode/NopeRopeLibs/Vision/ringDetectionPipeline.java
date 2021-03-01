package org.firstinspires.ftc.teamcode.NopeRopeLibs.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ringDetectionPipeline extends OpenCvPipeline{
    private static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(37,150);
    private static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(37,210);
    private static final int UPPER_ORANGE_THRESHOLD =  110;
    private static final int LOWER_ORANGE_THRESHOLD =  0;
    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static int loopCount = 0;
    private ElapsedTime timer = new ElapsedTime();
    private static double time = 0;

    private static final int REGION_WIDTH = 125;
    private static final int REGION_HEIGHT = 25;

    private Mat Cb = new Mat();
    private Mat YCrCb = new Mat();
    private Mat ringTop = new Mat();
    private Mat ringBot = new Mat();
    private static int avg1, avg2;
    public static int stackSize = -1;
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }


    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);

        ringTop = Cb.submat(new Rect(region1_pointA, region1_pointB));
        ringBot = Cb.submat(new Rect(region2_pointA, region2_pointB));
        timer.reset();
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);
        stackSize = 0;

        avg1 = (int) Core.mean(ringTop).val[0];
        avg2 = (int) Core.mean(ringBot).val[0];

        if(UPPER_ORANGE_THRESHOLD > avg1 && avg1 > LOWER_ORANGE_THRESHOLD){
            stackSize += 3;
        }
        if (UPPER_ORANGE_THRESHOLD > avg2 && avg2 > LOWER_ORANGE_THRESHOLD){
            stackSize ++;
        }

        Imgproc.rectangle(
                input,
                region1_pointA,
                region1_pointB,
                BLUE,
                2);

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Mat yCbCrChan2Mat = new Mat();
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);

        loopCount++;
        time = timer.seconds();
        return yCbCrChan2Mat;
    }
}
