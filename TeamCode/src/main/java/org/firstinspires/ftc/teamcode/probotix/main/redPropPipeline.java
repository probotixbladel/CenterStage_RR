package org.firstinspires.ftc.teamcode.probotix.main;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class redPropPipeline extends OpenCvPipeline
{
    //Tuneable Hue (blue)
    //Cone: 110,130  -  Line: 100,110
    public static double LowH = 330;
    public static double HighH = 360;
    //Tunable saturation
    public static double strictLowS = 50;
    public static double strictHighS = 255;

    private Telemetry telemetry = null;

    //Callable X and Y coordinates of the center of the stack
    int X = 0;
    int Y = 0;

    //some colours for visuals (RGB)
    Scalar White = new Scalar(255,255,255);
    Scalar Red = new Scalar(255,0,0);
    Scalar Green = new Scalar(0,255,0);
    Scalar Blue = new Scalar(0,0,255);
    Scalar Yellow = new Scalar(255,255,0);


    public redPropPipeline() {
        this.telemetry = telemetry;
    }

    //runs during init of OpenCV
    @Override
    public void init(Mat input)
    {}
    //runs every camera frame
    @Override
    public Mat processFrame(Mat input)
    {

        //Mat is an image
        //blur image to remove small details
        Mat blur = new Mat();
        Imgproc.GaussianBlur(input,blur, new Size(5,5),0);

        //convert image from RGB to HSV
        Mat HSV = new Mat();
        Imgproc.cvtColor(blur, HSV, Imgproc.COLOR_RGB2HSV);

        //get the lower and upper Saturation and lightness
        Scalar lowHSV = new Scalar(LowH,70,80);
        Scalar highHSV = new Scalar(HighH,255,255);

        //turns all pixels with values between lowHSV and highHSV white and all the others black
        Mat thresh = new Mat();
        Core.inRange(HSV,lowHSV,highHSV,thresh);

        //switches all the white pixels with the pixels one from HSV
        Mat masked = new Mat();
        Core.bitwise_and(HSV, HSV, masked,thresh);

        //Stricter saturation making it more exact and better tunable
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0);
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255);

        Mat scaledThresh = new Mat();
        Core.inRange(masked,strictLowHSV,strictHighHSV,scaledThresh);

        //list of contours. contour is a line drawn around the edges of a surface
        List<MatOfPoint> contours = new ArrayList<>();
        //necessary for function to work, wont be used
        Mat hierarchy = new Mat();
        //find contours, input must be in black and white for best outcome
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //draw all contours onto input (visual purpose only)
        //Imgproc.drawContours(input, contours, -1, Green,2);


        double area = 0;
        int index = 0;
        MatOfPoint biggestContour = null;

        //get the contour with the largest surface area and save it
        for (int i = 0; i < contours.size(); i++)
        {
            if(Imgproc.contourArea(contours.get(i)) > area) {
                area = Imgproc.contourArea(contours.get(i));
                biggestContour = contours.get(i);
                index = i;
            }
        }

        //return if nothing found to prevent crashes
        if(biggestContour == null)
        {
            return input;
        }

        //draw biggest contour (visual purpose only)
        //Imgproc.drawContours(input, contours, index, Green, 2);


        //create a rectangle around the contour
        Rect rect = Imgproc.boundingRect(biggestContour);

        //draw the created rectangle onto input
        Imgproc.rectangle(input,rect,Red,2);

        //display width and height of the rectangle (visual purpose only)
        //Imgproc.putText(input, String.valueOf(rect.x), new Point(rect.x - rect.width/2,rect.y + rect.height/5),Imgproc.FONT_HERSHEY_PLAIN,1,new Scalar(255,255,0),1);
        //Imgproc.putText(input, String.valueOf(rect.y), new Point(rect.x + rect.width/5,rect.y),Imgproc.FONT_HERSHEY_PLAIN,1,new Scalar(255,255,0),1);

        //Get coordinates of the centre of the rectangle
        X = rect.x + rect.width/2;
        Y = rect.y + rect.height/2;

        //Create text with the coordinates of the center of the rectangle
        String centerLocationString = String.format("(%d,%d)", X, Y);
        //Create a small rectangle in the center of the larger rectangle
        Rect center = new Rect(X, Y, 1,1);
        //draw the small rectangle on input
        Imgproc.rectangle(input,center,White,5);
        //draw the text on input
        Imgproc.putText(input, centerLocationString, new Point(X,Y),Imgproc.FONT_HERSHEY_PLAIN,5, Yellow,5);
        //draw the text of location 0,0 cause certain people cant handle coordinate systems very well ;)
        Imgproc.putText(input,"(0,0)", new Point(0,12),Imgproc.FONT_HERSHEY_PLAIN,5,Yellow,5);

        //release data of all non required Mats to save ram aka prevent crashing
        blur.release();
        HSV.release();
        thresh.release();
        masked.release();
        scaledThresh.release();
        hierarchy.release();

        //some telemetry for testing purposes
        /*
        telemetry.addData("Nr of contours: ", contours.size());
        telemetry.addData("Area of largest: ", area);
        for (MatOfPoint matOfPoint : contours) {
            telemetry.addData("Contour: ", matOfPoint);
        }
        telemetry.update();

         */
        //What is returned is displayed on the screen
        return input;
    }

    public int getX() {
        return X;
    }

    public int getY() {
        return Y;
    }
}