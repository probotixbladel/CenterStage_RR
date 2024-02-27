package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.probotix.main.StackPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.bluePropPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous(name="TestOdo", group="probotix")
public class TestOdo extends LinearOpMode {
    private hardware Hardware;
    OpenCvWebcam webcam;
    bluePropPipeline pipeline;
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new bluePropPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        int propXPos = 0;
        int trajNumber = 0;
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("X Location: ", pipeline.getX());
            //telemetry.addData("Y Location", pipeline.getY());
            telemetry.update();

            propXPos = pipeline.getX();

            sleep(50);
            if (propXPos<420){
                trajNumber = 1;
            }
            else if (propXPos>420){
                if (propXPos<850){
                    trajNumber =2;
                }
                else if(propXPos>850){
                    trajNumber = 3;
                }
            }
        }
        if (propXPos<420){
            trajNumber = 1;
        }
        else if (propXPos>420){
            if (propXPos<850){
                trajNumber =2;
            }
            else if(propXPos>850){
                trajNumber = 3;
            }
            telemetry.addData("traj:",trajNumber);
        }



        this.Hardware = new hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //forward -x, backward +x, left -y, right +y
        // We want to start the bot at x: 0, y: 0, heading: 0 degrees
        //Pose2d startPose = new Pose2d(62, 35, Math.toRadians(90));
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);




TrajectorySequence One = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(80, 0, Math.toRadians(0)))
                .build();

TrajectorySequence Two = drive.trajectorySequenceBuilder(One.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();
/*
TrajectorySequence Three = drive.trajectorySequenceBuilder(Two.end())
                .lineToLinearHeading(new Pose2d(0, 60, Math.toRadians(270)))
                .build();

TrajectorySequence Four = drive.trajectorySequenceBuilder(Three.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

/*TrajectorySequence Five = drive.trajectorySequenceBuilder(Four.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

TrajectorySequence Six = drive.trajectorySequenceBuilder(Five.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence Seven = drive.trajectorySequenceBuilder(Six.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

*/


        waitForStart();
        if (!isStopRequested()) {
          drive.followTrajectorySequence(One);
          sleep(100);
          drive.followTrajectorySequence(Two);
          sleep(100);
         // drive.followTrajectorySequence(Three);
          sleep(100);
        //  drive.followTrajectorySequence(Four);
          sleep(100);
          //drive.followTrajectorySequence(Five);
          sleep(100);
          //drive.followTrajectorySequence(Six);
          sleep(100);
          //drive.followTrajectorySequence(Seven);
        }}}
