package org.firstinspires.ftc.teamcode.probotix.main;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BasicPipeline extends OpenCvPipeline {

    /**
    TODO: Change later!*/
    public Point TOPLEFT_ANCHOR_POINT = new Point(60,60);
    public int REGION_WIDTH = 300;
    public int REGION_HEIGHT = 100;

    /*
    * Points which actually define the sample region rectangles, derived from above values
    *
    * Example of how points A and B work to define a rectangle
    *
    *   ------------------------------------
    *   | <-                               |  Anchor = point A = (20,20)
    *   |                                  |
    *   |                                  |
    *   |                                  |
    *   |                                  |
    *   |                                  |
    *   |                                  |
    *   |                               -> | point B = A + (width,height) = (90,70)
    *   ------------------------------------
    *
    */


 /* 
    private Telemetry telemetry;

    
    public BasicPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
*/


    /**
    enum values we'll use to return the right colour     */
    public enum SignalColour
    {          //test result values:
        BLUE,  //255
        WHITE, //128
        YELLOW //1
    }

    private volatile SignalColour Colour = SignalColour.WHITE;

    /**
    Mat is a variable where an image can be stored     */
    Mat regionCb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    public int avg;

    /**
    Runs on init     */
    @Override
    public void init(Mat firstFrame)
    {
        processFrame(firstFrame);
        
    }

    /**
    Runs everytime camera sends a frame     */
    @Override
    public Mat processFrame(Mat input)
    {
        /**
        Converting RGB input image to YCrCB (luma|blue chrome|red chroma)         */
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        /**
        Extracting the Cb value       */
        Core.extractChannel(YCrCb, Cb, 2);

        /** 
        Coordinates/points on a screen (in pixels)     */
        Point pointA = new Point(TOPLEFT_ANCHOR_POINT.x, TOPLEFT_ANCHOR_POINT.y);
        Point pointB = new Point(TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /**
        Crops Cb Mat to set rectangle values         */
        regionCb = Cb.submat(new Rect(pointA, pointB));

        /**
        Check the average value of the entire cropped area compared to blue         */
        avg = (int) Core.mean(regionCb).val[0];

        /**
        Draw a rectangle on the screen with no functionality to see the shape of the set rectangle        */
        Imgproc.rectangle(input,pointA,pointB,new Scalar(0,0,0),2);



        return input;

    }

 /* 
    @Override
    public void onViewportTapped() {

        telemetry.addData("avg", avg);
        telemetry.update();

    }
*/

    public SignalColour GetSignalColour()
    {

        if(avg < 65) {
            Colour = SignalColour.YELLOW;
        } else if (avg > 191) {
            Colour = SignalColour.BLUE;
        } else {
            Colour = SignalColour.WHITE;
        }

        return Colour;
    }

    public int getAvg() {
        return avg;
    }
}
