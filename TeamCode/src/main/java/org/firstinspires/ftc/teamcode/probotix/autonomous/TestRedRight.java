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
import org.firstinspires.ftc.teamcode.probotix.main.redPropPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous(name="TestRedRight", group="probotix")
public class TestRedRight extends LinearOpMode {
    private hardware Hardware;
    OpenCvWebcam webcam;
    redPropPipeline pipeline;
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new redPropPipeline();
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

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(15, -63.39, Math.toRadians(90.00)))
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .splineTo(new Vector2d(8.68, -32.46), Math.toRadians(180.00))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoOpen))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(48.80, -29.84), Math.toRadians(0.00))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+50))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(1650))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoPickUp))
                .splineTo(new Vector2d(17.29, -13.93), Math.toRadians(169.76))
                .splineTo(new Vector2d(-50.55, -12.33), Math.toRadians(0.00))
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(13.06, -16.71), Math.toRadians(-17.24))
                .splineTo(new Vector2d(48.51, -35.82), Math.toRadians(0.00))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(10))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(0))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoInit))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(15, -63.39, Math.toRadians(90.00)))
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .splineTo(new Vector2d(11.74, -32.75), Math.toRadians(449.24))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoOpen))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(48.95, -35.82), Math.toRadians(360.00))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+50))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(1650))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoPickUp))
                .splineTo(new Vector2d(4.89, -13.64), Math.toRadians(168.01))
                .splineTo(new Vector2d(-59.60, -11.89), Math.toRadians(360.00))
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(2.55, -12.91), Math.toRadians(352.28))
                .splineTo(new Vector2d(48.51, -29.25), Math.toRadians(360.00))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(10))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(0))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoInit))
                .waitSeconds(0.5)
                .build();


        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(15.00, -63.40, Math.toRadians(90.00)))
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .splineTo(new Vector2d(15.25, -31.44), Math.toRadians(0.00))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoOpen))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(24.15, -55.51), Math.toRadians(-22.77))
                .splineTo(new Vector2d(48.80, -42.53), Math.toRadians(0.00))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+50))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(1650))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoPickUp))
                .splineTo(new Vector2d(31.88, -15.68), Math.toRadians(166.75))
                .splineTo(new Vector2d(-12.77, -13.20), Math.toRadians(176.66))
                .splineTo(new Vector2d(-54.64, -11.74), Math.toRadians(0))
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-38.30, -12.47), Math.toRadians(1.12))
                .splineTo(new Vector2d(1.68, -13.79), Math.toRadians(2.32))
                .splineTo(new Vector2d(26.19, -16.85), Math.toRadians(20.00))
                .splineTo(new Vector2d(48.66, -35.96), Math.toRadians(0.00))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(10))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(0))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoInit))
                .waitSeconds(0.5)
                .build();









        waitForStart();
        webcam.stopStreaming();
        if (!isStopRequested()) {
            if(trajNumber == 1){
                drive.followTrajectorySequence(left);


            }
            else if(trajNumber == 2){
                drive.followTrajectorySequence(middle);

            }
            else{
                drive.followTrajectorySequence(right);


            }
        }
    }
}
