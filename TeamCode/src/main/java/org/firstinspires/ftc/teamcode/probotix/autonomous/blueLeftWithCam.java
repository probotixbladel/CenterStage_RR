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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous(name="blueLeftWithCam", group="probotix")
public class blueLeftWithCam extends LinearOpMode {
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


        //******************************** LEFT ******************************//

        TrajectorySequence deliverleft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, -13))
                .build();

        TrajectorySequence backupLeft = drive.trajectorySequenceBuilder(deliverleft.end())
                .lineToConstantHeading(new Vector2d(-20, -13))
                .build();

        TrajectorySequence deliverBackdropLeft = drive.trajectorySequenceBuilder(backupLeft.end())
                .lineToLinearHeading(new Pose2d(-22, -37, Math.toRadians(90)))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(deliverBackdropLeft.end())
                .lineToConstantHeading(new Vector2d(-5, -37))
                .build();

        //******************************** LEFT ******************************//

        //******************************** MIDDLE ****************************//

        TrajectorySequence deliverMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-33, 0))
                .build();

        TrajectorySequence backupMiddle = drive.trajectorySequenceBuilder(deliverMiddle.end())
                .lineToConstantHeading(new Vector2d(-28,0))
                .build();

        TrajectorySequence deliverBackdropMiddle = drive.trajectorySequenceBuilder(backupMiddle.end())
                .lineToLinearHeading(new Pose2d(-30,-37,Math.toRadians(90)))
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(deliverBackdropMiddle.end())
                .lineToConstantHeading(new Vector2d(-50, -37))
                .build();

        //******************************** MIDDLE ****************************//

        //**************************** RIGHT *************************************//

        TrajectorySequence deliverRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-28,0))
                .build();

        TrajectorySequence backupRight = drive.trajectorySequenceBuilder(deliverRight.end())
                .lineToLinearHeading(new Pose2d(-30,6, Math.toRadians(-90)))
                .build();

        TrajectorySequence deliverBackdropRight = drive.trajectorySequenceBuilder(backupRight.end())
                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(90)))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(deliverBackdropRight.end())
                .lineToConstantHeading(new Vector2d(-50, -37))
                .build();

        //**************************** RIGHT *************************************//


        waitForStart();

        if (!isStopRequested()) {
        if(trajNumber == 1){
            drive.followTrajectorySequence(deliverleft);
            Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
            sleep(1000);
            drive.followTrajectorySequence(backupLeft);
            Hardware.dropServo.setPosition(DriveConstants.dropServoClose);


            drive.followTrajectorySequence(deliverBackdropLeft);
            drive.followTrajectorySequence(parkLeft);
            Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
            sleep(500);
        }
        else if(trajNumber == 2){
            drive.followTrajectorySequence(deliverMiddle);
            Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
            sleep(1000);
            drive.followTrajectorySequence(backupMiddle);
            Hardware.dropServo.setPosition(DriveConstants.dropServoClose);


            drive.followTrajectorySequence(deliverBackdropMiddle);
            drive.followTrajectorySequence(parkMiddle);
            Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
            sleep(500);
        }
        else{
            drive.followTrajectorySequence(deliverRight);
            drive.followTrajectorySequence(backupRight);
            Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
            sleep(1000);
            drive.followTrajectorySequence(deliverBackdropRight);
            Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
            drive.followTrajectorySequence(parkRight);
            Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
            sleep(500);
        }
        }
    }
}
