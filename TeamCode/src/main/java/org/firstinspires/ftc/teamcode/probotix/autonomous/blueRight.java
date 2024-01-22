package org.firstinspires.ftc.teamcode.probotix.autonomous;

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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;


@Autonomous(name="blueRight", group="probotix")
public class blueRight extends LinearOpMode {
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

        //***************************** LEFT ***************************************//

        TrajectorySequence deliverleft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-28, 0))
                .build();

        TrajectorySequence backupLeft = drive.trajectorySequenceBuilder(deliverleft.end())
                .lineToLinearHeading(new Pose2d(-28,-8, Math.toRadians(90)))
                .build();

TrajectorySequence goAlittleBack = drive.trajectorySequenceBuilder(backupLeft.end())
        .lineToLinearHeading(new Pose2d(-28,-6, Math.toRadians(90)))
        .build();
        TrajectorySequence backLeft = drive.trajectorySequenceBuilder(goAlittleBack.end())
                .lineToLinearHeading(new Pose2d(-28,0,Math.toRadians(90)))
                .build();

        TrajectorySequence goBackLeft = drive.trajectorySequenceBuilder(backLeft.end())
                .lineToLinearHeading(new Pose2d(-3,0,Math.toRadians(90)))
                .build();

        TrajectorySequence deliverBackdropLeft = drive.trajectorySequenceBuilder(goBackLeft.end())
                .lineToLinearHeading(new Pose2d(-3, -80, Math.toRadians(90)))
                .build();

        TrajectorySequence correctLeft = drive.trajectorySequenceBuilder(deliverBackdropLeft.end())
                .lineToLinearHeading(new Pose2d(-22,-88,Math.toRadians(90)))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(correctLeft.end())
                .lineToLinearHeading(new Pose2d(-3, -88,Math.toRadians(90)))
                .build();
        //***************************** LEFT ***************************************//


        //**************************** MIDDLE **************************************//

        TrajectorySequence deliverMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-32,0))
                .build();

        TrajectorySequence backupMiddle = drive.trajectorySequenceBuilder(deliverMiddle.end())
                .lineToLinearHeading(new Pose2d(-3,0, Math.toRadians(90)))
                .build();

        TrajectorySequence deliverBackdropMiddle = drive.trajectorySequenceBuilder(backupMiddle.end())
                .lineToLinearHeading(new Pose2d(-3,-87,Math.toRadians(90)))
                .build();

        TrajectorySequence correctMiddle = drive.trajectorySequenceBuilder(deliverBackdropMiddle.end())
                .lineToLinearHeading(new Pose2d(-30,-89,Math.toRadians(90)))
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(correctMiddle.end())
                .lineToConstantHeading(new Vector2d(-55, -89))
                .build();

        //**************************** MIDDLE **************************************//


        //***************************** RIGHT **************************************//

        TrajectorySequence deliverRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30,13))
                .build();

        TrajectorySequence backupRight = drive.trajectorySequenceBuilder(deliverRight.end())
                .lineToLinearHeading(new Pose2d(-4, 13,Math.toRadians(90)))
                .build();

        TrajectorySequence goBackRight = drive.trajectorySequenceBuilder(backupRight.end())
                .lineToLinearHeading(new Pose2d(-4, 0, Math.toRadians(90)))
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(goBackRight.end())
                .lineToLinearHeading(new Pose2d(-4, -80, Math.toRadians(90)))
                .build();

        TrajectorySequence deliverBackdropRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .lineToConstantHeading(new Vector2d(-38, -85))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(deliverBackdropRight.end())
                .lineToConstantHeading(new Vector2d(-57,-85))
                        .build();


        //***************************** RIGHT **************************************//



        waitForStart();
        webcam.stopStreaming();
        if (!isStopRequested()) {
            if(trajNumber == 1){
                Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
                sleep(10);
                drive.followTrajectorySequence(deliverleft);
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                sleep(10);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+100);
                sleep(500);
                Hardware.armMotor.setTargetPosition(DriveConstants.armPickUp-100);
                sleep(500);
                drive.followTrajectorySequence(backupLeft);
                drive.followTrajectorySequence(goAlittleBack);
                Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
                sleep(500);
                drive.followTrajectorySequence(backLeft);
                drive.followTrajectorySequence(goBackLeft);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
                drive.followTrajectorySequence(deliverBackdropLeft);
                drive.followTrajectorySequence(correctLeft);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                sleep(1000);
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
                sleep(10);
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+100);
                sleep(500);
                Hardware.armMotor.setTargetPosition(DriveConstants.armPickUp-100);
                sleep(500);
                drive.followTrajectorySequence(backupMiddle);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);

                drive.followTrajectorySequence(deliverBackdropMiddle);
                drive.followTrajectorySequence(correctMiddle);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                sleep(1000);
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
                sleep(500);
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
                sleep(50);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+100);
                sleep(500);
                Hardware.armMotor.setTargetPosition(DriveConstants.armPickUp-100);
                sleep(500);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
                drive.followTrajectorySequence(backupRight);
                drive.followTrajectorySequence(goBackRight);
                drive.followTrajectorySequence(goToBackdrop);
                drive.followTrajectorySequence(deliverBackdropRight);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                sleep(1000);
                Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
                sleep(500);
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
