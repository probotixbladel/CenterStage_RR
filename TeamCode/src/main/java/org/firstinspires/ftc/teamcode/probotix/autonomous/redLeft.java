package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.probotix.main.StackPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.redPropPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="redLeft", group="probotix")
public class redLeft extends LinearOpMode {
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

        //***************************** RIGHT ***************************************//

        TrajectorySequence deliverRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-28, 0))
                .build();

        TrajectorySequence backupRight = drive.trajectorySequenceBuilder(deliverRight.end())
                .lineToLinearHeading(new Pose2d(-30,5, Math.toRadians(-90)))
                .build();
        TrajectorySequence goALittleBack = drive.trajectorySequenceBuilder(backupRight.end())
                .lineToLinearHeading(new Pose2d(-28, 5, Math.toRadians(-90)))
                .build();
        TrajectorySequence backRight = drive.trajectorySequenceBuilder(goALittleBack.end())
                .lineToLinearHeading(new Pose2d(-28, 0,Math.toRadians(-90)))
                .build();
        TrajectorySequence goBackRight = drive.trajectorySequenceBuilder(backRight.end())
                .lineToLinearHeading(new Pose2d(-3,0,Math.toRadians(-90)))
                .build();

        TrajectorySequence deliverBackdropRight = drive.trajectorySequenceBuilder(goBackRight.end())
                .lineToLinearHeading(new Pose2d(-3, 80, Math.toRadians(-90)))
                .build();

        TrajectorySequence correctRight = drive.trajectorySequenceBuilder(deliverBackdropRight.end())
                .lineToLinearHeading(new Pose2d(-21, 87, Math.toRadians(-90)))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(correctRight.end())
                .lineToLinearHeading(new Pose2d(-3, 84,Math.toRadians(-90)))
                .build();
        //***************************** RIGHT ***************************************//


        //**************************** MIDDLE **************************************//

        TrajectorySequence deliverMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-32,0))
                .build();

        TrajectorySequence backupMiddle = drive.trajectorySequenceBuilder(deliverMiddle.end())
                .lineToLinearHeading(new Pose2d(-3,0, Math.toRadians(-90)))
                .build();

        TrajectorySequence deliverBackdropMiddle = drive.trajectorySequenceBuilder(backupMiddle.end())
                .lineToLinearHeading(new Pose2d(-3,80,Math.toRadians(-90)))
                .build();

        TrajectorySequence correctMiddle = drive.trajectorySequenceBuilder(deliverBackdropMiddle.end())
                .lineToLinearHeading(new Pose2d(-29,88,Math.toRadians(-90)))
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(correctMiddle.end())
                .lineToConstantHeading(new Vector2d(-52, 85))
                .build();

        //**************************** MIDDLE **************************************//


        //***************************** LEFT **************************************//

        TrajectorySequence deliverLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30,-12))
                .build();

        TrajectorySequence backupLeft = drive.trajectorySequenceBuilder(deliverLeft.end())
                .lineToLinearHeading(new Pose2d(-4, -13,Math.toRadians(-90)))
                .build();

        TrajectorySequence goBackLeft = drive.trajectorySequenceBuilder(backupLeft.end())
                .lineToLinearHeading(new Pose2d(-4, 80, Math.toRadians(-90)))
                .build();


        TrajectorySequence correctLeft = drive.trajectorySequenceBuilder(goBackLeft.end())
                .lineToConstantHeading(new Vector2d(-37, 85))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(correctLeft.end())
                .lineToConstantHeading(new Vector2d(-52, 84))
                .build();

        //***************************** RIGHT **************************************//


        waitForStart();
        webcam.stopStreaming();
        if (!isStopRequested()) {
            if(trajNumber == 1){
                Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
                sleep(10);
                drive.followTrajectorySequence(deliverLeft);
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
                sleep(100);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+100);
                sleep(500);
                Hardware.armMotor.setTargetPosition(DriveConstants.armPickUp-100);
                sleep(500);
                drive.followTrajectorySequence(backupLeft);
                drive.followTrajectorySequence(goBackLeft);
                drive.followTrajectorySequence(correctLeft);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
                sleep(500);
                Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
                sleep(700);
                Hardware.armMotor.setTargetPosition(0);
                sleep(100);
                Hardware.liftMotor.setTargetPosition(10);
                Hardware.flipServo.setPosition(DriveConstants.flipServoInit);
                sleep(200);
                drive.followTrajectorySequence(parkLeft);
            }
            else if(trajNumber == 2){
                Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
                sleep(10);
                drive.followTrajectorySequence(deliverMiddle);
                Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
                sleep(500);
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+100);
                sleep(500);
                Hardware.armMotor.setTargetPosition(DriveConstants.armPickUp-200);
                sleep(500);
                drive.followTrajectorySequence(backupMiddle);
                drive.followTrajectorySequence(deliverBackdropMiddle);
                drive.followTrajectorySequence(correctMiddle);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
                sleep(500);
                Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
                sleep(700);
                Hardware.armMotor.setTargetPosition(0);
                sleep(100);
                Hardware.liftMotor.setTargetPosition(10);
                Hardware.flipServo.setPosition(DriveConstants.flipServoInit);
                sleep(200);
                drive.followTrajectorySequence(parkMiddle);

            }
            else{
                Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
                sleep(10);
                drive.followTrajectorySequence(deliverRight);
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+100);
                sleep(500);
                Hardware.armMotor.setTargetPosition(DriveConstants.armPickUp-100);
                sleep(500);
                drive.followTrajectorySequence(backupRight);
                Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
                drive.followTrajectorySequence(goALittleBack);
                sleep(500);
                drive.followTrajectorySequence(backRight);
                drive.followTrajectorySequence(goBackRight);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
                drive.followTrajectorySequence(deliverBackdropRight);
                drive.followTrajectorySequence(correctRight);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                sleep(500);
                Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
                sleep(700);
                Hardware.armMotor.setTargetPosition(0);
                sleep(100);
                Hardware.liftMotor.setTargetPosition(10);
                Hardware.flipServo.setPosition(DriveConstants.flipServoInit);
                sleep(200);
                drive.followTrajectorySequence(parkRight);
            }
        }
    }
}