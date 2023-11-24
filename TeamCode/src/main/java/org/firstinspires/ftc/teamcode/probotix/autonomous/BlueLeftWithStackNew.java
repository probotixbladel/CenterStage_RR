package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.examples.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="LeftWithStackNew", group="probotix")
public class BlueLeftWithStackNew extends LinearOpMode{
    private hardware Hardware;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int lastId = 0;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;



    @Override
    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        //forward -x, backward +x, left -y, right +y
        // We want to start the bot at x: 0, y: 0, heading: 0 degrees
        //Pose2d startPose = new Pose2d(62, 35, Math.toRadians(90));
        Pose2d startPose = new Pose2d(105.5, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pushCone = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(105.5,-4))
                .build();


        TrajectorySequence deliverCone = drive.trajectorySequenceBuilder(pushCone.end())
                .forward(10)
                    //100.6 .5
                .splineToLinearHeading(new Pose2d(104,2,Math.toRadians(45)),Math.toRadians(45))
                .build();

        TrajectorySequence moveToStack = drive.trajectorySequenceBuilder(deliverCone.end())
                .setReversed(true)
                .addTemporalMarker(1,()->{
                    Hardware.liftMotor.setTargetPosition(-1700);
                    Hardware.liftMotor.setPower(0.8);
                })
                .forward(6)
                //.splineToLinearHeading(new Pose2d(28,11.5,Math.toRadians(5)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(112.5,14,Math.toRadians(180)),Math.toRadians(180))
                //.splineToSplineHeading(new Pose2d(26,11.5,Math.toRadians(0)),Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(14.4,9.9),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(130 ,14),Math.toRadians(180))
                .build();



        TrajectorySequence scoreSecondCone = drive.trajectorySequenceBuilder(moveToStack.end())
                .setReversed(false)

                .splineToConstantHeading(new Vector2d(120,11),Math.toRadians(0))
                .addTemporalMarker(1,()->{
                    Hardware.liftMotor.setTargetPosition(-4180);
                    Hardware.liftMotor.setPower(0.8);
                })
                .splineToLinearHeading(new Pose2d(95,11,Math.toRadians(0)),Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(80.2,20,Math.toRadians(-40)),Math.toRadians(-40))
                //Math.toRadians(-135)),Math.toRadians(-135))

                .build();


        TrajectorySequence getThirdCone = drive.trajectorySequenceBuilder(scoreSecondCone.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(90,14,Math.toRadians(180)))
                .addTemporalMarker(1,()->{
                    Hardware.liftMotor.setTargetPosition(-1700);
                    Hardware.liftMotor.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(133,15.2))
                .addTemporalMarker(2,()->{
                    Hardware.liftMotor.setTargetPosition(-1050);
                    Hardware.liftMotor.setPower(0.8);
                })
                .build();





        TrajectorySequence park1 = drive.trajectorySequenceBuilder(getThirdCone.end())
                .setReversed(true)
                //.lineToConstantHeading(new Vector2d(129.5,12))
                .lineToLinearHeading(new Pose2d(133,30,Math.toRadians(90)))

                .addTemporalMarker(1,()->{
                    Hardware.liftMotor.setTargetPosition(0);
                    Hardware.liftMotor.setPower(0.8);
                })
                .build();




        TrajectorySequence park2 = drive.trajectorySequenceBuilder(getThirdCone.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(109,13))
                .addTemporalMarker(1,()->{
                    Hardware.liftMotor.setTargetPosition(0);
                    Hardware.liftMotor.setPower(0.8);
                })

                .build();



        TrajectorySequence park3 = drive.trajectorySequenceBuilder(getThirdCone.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(84.1,13))

                .addTemporalMarker(1.5,()->{
                    Hardware.liftMotor.setTargetPosition(0);
                    Hardware.liftMotor.setPower(0.8);
                })
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode){}
        });




        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //dashboard.startCameraStream(webcam, 10);
        //Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while(!opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                // If we don't see any tags
                if(detections.size() == 0)
                {
                    telemetry.addLine("Tag not visible");
                    telemetry.update();
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        lastId = detection.id;
                    }
                }


            }

            if (lastId == 0)
            {
                telemetry.addLine("It's 0");
            }
            else if (lastId == 2)
            {
                telemetry.addLine("It's 2");
            }
            else if (lastId == 4)
            {
                telemetry.addLine("It's 4");
            }

            telemetry.update();

            sleep(20);

        }


        waitForStart();

        //telemetry.setMsTransmissionInterval(50);



        if (!isStopRequested()) {
            Hardware.liftMotor.setTargetPosition(-2200);
            Hardware.liftMotor.setPower(0.7);

            drive.followTrajectorySequence(pushCone);





            Hardware.liftMotor.setTargetPosition(-4180);
            Hardware.liftMotor.setPower(0.8);

            drive.followTrajectorySequence(deliverCone);





            sleep(1500);
            Hardware.grabServo.setPosition(0.70);
            sleep(500);






            drive.followTrajectorySequence(moveToStack);




            Hardware.liftMotor.setTargetPosition(-560);
            Hardware.liftMotor.setPower(0.8);
            sleep(800);
            Hardware.grabServo.setPosition(0);
            sleep(700);

            Hardware.liftMotor.setTargetPosition(-1700);
            Hardware.liftMotor.setPower(0.8);
            sleep(700);


            drive.followTrajectorySequence(scoreSecondCone);

            //sleep(5000);


            sleep(1500);
            Hardware.grabServo.setPosition(0.70);
            sleep(500);




            drive.followTrajectorySequence(getThirdCone);



            Hardware.liftMotor.setTargetPosition(-480);
            Hardware.liftMotor.setPower(0.8);
            sleep(800);


            Hardware.grabServo.setPosition(0);
            sleep(700);

            Hardware.liftMotor.setTargetPosition(-1700);
            Hardware.liftMotor.setPower(0.8);
            sleep(700);



            if (lastId == 0) {
                drive.followTrajectorySequence(park1);
            } else if (lastId == 2) {
                drive.followTrajectorySequence(park2);
            } else if (lastId == 4) {
                drive.followTrajectorySequence(park3);
            }

            sleep(1000);


        }
    }
}

