package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.concurrent.atomic.AtomicBoolean;


public class RedVisionPipeline2 extends OpenCvPipeline {

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

    static final Point CENTER_REGION_ANCHOR_POINT = new Point(650 * X_SCALE_FACTOR,560 * Y_SCALE_FACTOR);//was 500, 105
    static final Point LEFT_REGION_ANCHOR_POINT = new Point(1125 * X_SCALE_FACTOR, 560 * Y_SCALE_FACTOR);

    static final int REGION_WIDTH = (int) (10 * X_SCALE_FACTOR);
    static final int REGION_HEIGHT = (int) (25 * Y_SCALE_FACTOR);

    Point centerRegion_pointA = new Point(CENTER_REGION_ANCHOR_POINT.x, CENTER_REGION_ANCHOR_POINT.y);
    Point centerRegion_pointB = new Point(CENTER_REGION_ANCHOR_POINT.x + REGION_WIDTH, CENTER_REGION_ANCHOR_POINT.y + REGION_HEIGHT);

    Point leftRegion_pointA = new Point(LEFT_REGION_ANCHOR_POINT.x, LEFT_REGION_ANCHOR_POINT.y);
    Point leftRegion_pointB = new Point(LEFT_REGION_ANCHOR_POINT.x + REGION_WIDTH, LEFT_REGION_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat center_region_H, center_region_S;
    Mat left_region_H, left_region_S;
    Mat HSV = new Mat();
    Mat Hue = new Mat();
    Mat Saturation = new Mat();

    double center_region_H_average, center_region_S_average, left_region_H_average, left_region_S_average;

    // variable to keep track of if a call to processFrame() has completed.
    // since processFrame() seems to be called from another thread, this variable is atomic to make
    // it thread safe.
    private final AtomicBoolean hasCompletedAnalysis;

    public RedVisionPipeline2() {
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
        center_region_H = Hue.submat(new Rect(centerRegion_pointA, centerRegion_pointB));
        left_region_H = Hue.submat(new Rect(leftRegion_pointA, leftRegion_pointB));

        center_region_S = Saturation.submat(new Rect (centerRegion_pointA, centerRegion_pointB));
        left_region_S = Saturation.submat(new Rect (leftRegion_pointA,leftRegion_pointB));

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
        center_region_H_average = (int) Core.mean(center_region_H).val[0];
        left_region_H_average = (int) Core.mean(left_region_H).val[0];

        center_region_S_average = (int) Core.mean(center_region_S).val[0];
        left_region_S_average = (int) Core.mean(left_region_S).val[0];

        double hMinimum = 20;//this was 30
        double hMaximum = 60;//this was 50

        double sMinimum = 70;//this was 120
        double sMaximum = 220;//this was 200
        /*
         * Draw rectangles showing the three regions on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                centerRegion_pointA, // First point which defines the rectangle
                centerRegion_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                leftRegion_pointA, // First point which defines the rectangle
                leftRegion_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if (center_region_H_average >= hMinimum && center_region_H_average <= hMaximum && center_region_S_average >= sMinimum && center_region_S_average <= sMaximum){
            position = DuckPosition.CENTER;
        } else if (left_region_H_average >= hMinimum && left_region_H_average <= hMaximum && left_region_S_average >= sMinimum && left_region_S_average <= sMaximum){
            position = DuckPosition.LEFT;
        } else {
            position = DuckPosition.RIGHT;
        }
        hasCompletedAnalysis.set(true);

        RobotLog.dd(TAG, "Pattern: %s", position.toString());
        RobotLog.vv(TAG, "UpperRegionHAverage: %.5f", center_region_H_average);
        RobotLog.vv(TAG, "LowerRegionHAverage: %.5f", left_region_H_average);
        RobotLog.vv(TAG, "UpperRegionSAverage: %.5f", center_region_S_average);
        RobotLog.vv(TAG, "LowerRegionSAverage: %.5f", left_region_S_average);


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



