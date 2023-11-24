package org.firstinspires.ftc.teamcode.probotix.main;

import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AdvancedPipeline extends OpenCvPipeline
{

/* 
    private Telemetry telemetry;

    
    public DeterminationPipeline_OLD(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
*/

    public static enum SignalColour
    {
        RED,
        GREEN,
        BLUE

    }

    private volatile SignalColour Colour = SignalColour.GREEN;
    
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat mat2 = new Mat();
    int location = 0;

    private void blur(Mat input, double doubleRadius, Mat output) {

		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
        kernelSize = 6 * radius + 1;
		Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);

	}

    private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
			new Scalar(hue[1], lum[1], sat[1]), out);
	}

    private void mask(Mat input, Mat mask, Mat output) {
		mask.convertTo(mask, CvType.CV_8UC1);
		Core.bitwise_xor(output, output, output);
		input.copyTo(output, mask);
	}

    private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
		Core.inRange(out, new Scalar(blue[0], green[0], red[0]),
			new Scalar(blue[1], green[1], red[1]), out);
	}

    private Mat blurOutput = new Mat();
	private Mat hslThresholdOutput = new Mat();
	private Mat maskOutput = new Mat();
	private Mat rgbThresholdOutput = new Mat();

    private double maxVal;

    private double findMinAndMax(Mat src) {
		MinMaxLocResult data = Core.minMaxLoc(src);
        return data.maxVal;
	}

    void check(Mat input, int threshRed, int threshGreen, int threshBlue, int attempt)
    {

        int redUpperThresh = 0;
        int greenUpperThresh = 0;
        int blueUpperThresh = 0;

        switch(attempt){
            case 1://red
            redUpperThresh = 255;
            break;

            case 2://green
            greenUpperThresh = 255;
            break;

            case 3://blue
            blueUpperThresh = 255;
            break;
        }

        Mat rgbThresholdInput = maskOutput;
		double[] rgbThresholdRed = {threshRed, redUpperThresh};
		double[] rgbThresholdGreen = {threshGreen, greenUpperThresh};
		double[] rgbThresholdBlue = {threshBlue, blueUpperThresh};
		rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);
        
        Mat findMinAndMaxImage = rgbThresholdOutput;
		maxVal = findMinAndMax(findMinAndMaxImage);

        if(threshRed > 50 && maxVal > 120) {
            Colour = SignalColour.RED;
        }
        else if(threshGreen > 50 && maxVal > 120) {
            Colour = SignalColour.GREEN;
        }
        else if(threshBlue > 50 && maxVal > 120) {
            Colour = SignalColour.BLUE;
        }

    }

    Mat process(Mat input)
    {

        Mat blurInput = input;
		double blurRadius = 3.5;
		blur(blurInput, blurRadius, blurOutput);

         
		Mat hslThresholdInput = blurOutput;
		double[] hslThresholdHue = {0.0, 130.0};
		double[] hslThresholdSaturation = {208.0, 255.0};
		double[] hslThresholdLuminance = {26.0, 137.5};
		hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);

        Mat maskInput = input;
		Mat maskMask = hslThresholdOutput;
		mask(maskInput, maskMask, maskOutput);

        check(maskOutput, 128, 0, 0,1);

        check(maskOutput, 0, 128, 0,2);

        check(maskOutput, 0, 0, 128,3);

        return maskOutput;
    }

    @Override
    public void init(Mat input)
    {
        processFrame(input);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        input = process(input);

/* 
        switch(Colour){

            case RED:
            telemetry.addLine("Colour: RED");
            break;

            case GREEN:
            telemetry.addLine("Colour: GREEN");
            break;

            case BLUE:
            telemetry.addLine("Colour: BLUE");
            break;
            
        }
        telemetry.update();
*/

        return input;
    }

    public SignalColour getColour()
    {
        return Colour;
    }
}