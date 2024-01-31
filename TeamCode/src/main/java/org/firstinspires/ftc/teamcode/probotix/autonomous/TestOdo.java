package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
    public void runOpMode() throws InterruptedException {


       
       
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("X Location: ", pipeline.getX());
            telemetry.addData("Y Location", pipeline.getY());
            telemetry.update();

            propXPos = pipeline.getX();

            sleep(50);
           
        }}



        this.Hardware = new hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //forward -x, backward +x, left -y, right +y
        // We want to start the bot at x: 0, y: 0, heading: 0 degrees
        //Pose2d startPose = new Pose2d(62, 35, Math.toRadians(90));
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

TrajectorySequence 1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-60, 0, Math.toRadians(0)))
                .build();

TrajectorySequence 2 = drive.trajectorySequenceBuilder(1.end())
                .lineToLinearHeading(new Pose2d(-60, 30, Math.toRadians(180)))
                .build();

TrajectorySequence 3 = drive.trajectorySequenceBuilder(2.end())
                .lineToLinearHeading(new Pose2d(0, 30, Math.toRadians(180)))
                .build();

TrajectorySequence 4 = drive.trajectorySequenceBuilder(3.end())
                .lineToLinearHeading(new Pose2d(0, 60, Math.toRadians(0)))
                .build();

TrajectorySequence 5 = drive.trajectorySequenceBuilder(4.end())
                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(0)))
                .build();

TrajectorySequence 6 = drive.trajectorySequenceBuilder(5.end())
                .lineToLinearHeading(new Pose2d(-60, 90, Math.toRadians(180)))
                .build();

TrajectorySequence 7 = drive.trajectorySequenceBuilder(6.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();




        waitForStart();
        if (!isStopRequested()) {
          drive.followTrajectorySequence(1);
          sleep(100);
          drive.followTrajectorySequence(2);
          sleep(100);
          drive.followTrajectorySequence(3);
          sleep(100);
          drive.followTrajectorySequence(4);
          sleep(100);
          drive.followTrajectorySequence(5);
          sleep(100);
          drive.followTrajectorySequence(6);
          sleep(100);
          drive.followTrajectorySequence(7);
        }
