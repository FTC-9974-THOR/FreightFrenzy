package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.concurrent.atomic.AtomicBoolean;


public class RedVisionPipeline extends OpenCvPipeline {

    private static final String TAG = "FFDetectionPipeline";

    public enum DuckPosition{
        LEFT,
        CENTER,
        RIGHT
    }

    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0,0,255);

    DuckPosition position = null;

    // since I had to change the streaming resolution, I had to scale each point, width, and height
    // to match. each point goes to the same place in the image; the image is just at a lower
    // resolution.
    private static final double X_SCALE_FACTOR = 640.0 / 1280.0;
    private static final double Y_SCALE_FACTOR = 480.0 / 720.0;

    static final Point LEFT_REGION_ANCHOR_POINT = new Point(650 * X_SCALE_FACTOR,400 * Y_SCALE_FACTOR);//was 500, 105
    static final Point MIDDLE_REGION_ANCHOR_POINT = new Point(950 * X_SCALE_FACTOR, 400* Y_SCALE_FACTOR);

    static final int REGION_WIDTH = (int) (10 * X_SCALE_FACTOR);
    static final int REGION_HEIGHT = (int) (25 * Y_SCALE_FACTOR);

    Point leftRegion_pointA = new Point(LEFT_REGION_ANCHOR_POINT.x, LEFT_REGION_ANCHOR_POINT.y);
    Point leftRegion_pointB = new Point(LEFT_REGION_ANCHOR_POINT.x + REGION_WIDTH, LEFT_REGION_ANCHOR_POINT.y + REGION_HEIGHT);

    Point middleRegion_pointA = new Point(MIDDLE_REGION_ANCHOR_POINT.x, MIDDLE_REGION_ANCHOR_POINT.y);
    Point middleRegion_pointB = new Point(MIDDLE_REGION_ANCHOR_POINT.x + REGION_WIDTH, MIDDLE_REGION_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat left_region_H, left_region_S;
    Mat middle_region_H, middle_region_S;
    Mat HSV = new Mat();
    Mat Hue = new Mat();
    Mat Saturation = new Mat();

    double left_region_H_average, left_region_S_average, middle_region_H_average,middle_region_S_average;

    // variable to keep track of if a call to processFrame() has completed.
    // since processFrame() seems to be called from another thread, this variable is atomic to make
    // it thread safe.
    private final AtomicBoolean hasCompletedAnalysis;

    public RedVisionPipeline() {
        // initialize hasCompletedAnalysis
        hasCompletedAnalysis = new AtomicBoolean(false);
    }

    void inputToH(Mat input)
    {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, Hue, 0);
    }

    void inputToS(Mat input)
    {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, Saturation,1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToH(firstFrame);
        inputToS(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        left_region_H = Hue.submat(new Rect(leftRegion_pointA, leftRegion_pointB));
        middle_region_H = Hue.submat(new Rect(middleRegion_pointA, middleRegion_pointB));

        left_region_S = Saturation.submat(new Rect (leftRegion_pointA, leftRegion_pointB));
        middle_region_S = Saturation.submat(new Rect (middleRegion_pointA, middleRegion_pointB));

    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToH(input);
        inputToS(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        left_region_H_average = (int) Core.mean(left_region_H).val[0];
        middle_region_H_average = (int) Core.mean(middle_region_H).val[0];

        left_region_S_average = (int) Core.mean(left_region_S).val[0];
        middle_region_S_average = (int) Core.mean(middle_region_S).val[0];

        double hMinimum = 30;
        double hMaximum = 50;//this was 29

        double sMinimum = 120;//this was 50
        double sMaximum = 200;//this was 255
        /*
         * Draw rectangles showing the three regions on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                leftRegion_pointA, // First point which defines the rectangle
                leftRegion_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                middleRegion_pointA, // First point which defines the rectangle
                middleRegion_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if (left_region_H_average >= hMinimum && left_region_H_average <= hMaximum && left_region_S_average >= sMinimum && left_region_S_average <= sMaximum){
            position = DuckPosition.LEFT;
        } else if (middle_region_H_average >= hMinimum && middle_region_H_average <= hMaximum && middle_region_S_average >= sMinimum && middle_region_S_average <= sMaximum){
            position = DuckPosition.CENTER;
        } else {
            position = DuckPosition.RIGHT;
        }
        hasCompletedAnalysis.set(true);

        RobotLog.dd(TAG, "Pattern: %s", position.toString());
        RobotLog.vv(TAG, "UpperRegionHAverage: %.5f", left_region_H_average);
        RobotLog.vv(TAG, "LowerRegionHAverage: %.5f", middle_region_H_average);
        RobotLog.vv(TAG, "UpperRegionSAverage: %.5f", left_region_S_average);
        RobotLog.vv(TAG, "LowerRegionSAverage: %.5f", middle_region_S_average);


        return input;
    }

    public boolean hasAnalysis() {
        return hasCompletedAnalysis.get();
    }

    public DuckPosition getAnalysis()
    {
        return position;
    }
}



