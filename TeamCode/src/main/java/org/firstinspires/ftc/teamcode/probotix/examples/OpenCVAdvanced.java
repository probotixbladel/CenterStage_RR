package org.firstinspires.ftc.teamcode.probotix.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.main.BasicPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name="OpenCVAdvanced", group="probotix")
public class OpenCVAdvanced extends LinearOpMode{
    private hardware Hardware;
    //AdvancedPipeline pipeline = new AdvancedPipeline();
    BasicPipeline pipeline = new BasicPipeline();
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode){}
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        //dashboard.startCameraStream(webcam, 10);
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while(!opModeIsActive())
        {

            int avg = pipeline.getAvg();
            telemetry.addData("val:", avg);


            switch(pipeline.GetSignalColour()){

                case YELLOW:
                    telemetry.addLine("Colour: YELLOW");
                    dashboardTelemetry.addLine("Colour: YELLOW");
                    break;

                case BLUE:
                    telemetry.addLine("Colour: BLUE");
                    dashboardTelemetry.addLine("Colour: BLUE");
                    break;

                case WHITE:
                    telemetry.addLine("Colour: WHITE");
                    dashboardTelemetry.addLine("Colour: WHITE");
                    break;

            }
            telemetry.update();
            dashboardTelemetry.update();
            sleep(50);
        }


        waitForStart();

        if (!isStopRequested());


    }
}
