package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.main.StackPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="blueLeftWithCam", group="probotix")
public class blueLeftWithCam extends LinearOpMode {
    private hardware Hardware;
    OpenCvWebcam webcam;
    StackPipeline pipeline;
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new StackPipeline();
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
            telemetry.addData("Y Location", pipeline.getY());
            telemetry.addData("trajectory:", trajNumber);
            telemetry.update();

            propXPos = pipeline.getX();

            sleep(50);
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
        }



            this.Hardware = new hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //forward -x, backward +x, left -y, right +y
        // We want to start the bot at x: 0, y: 0, heading: 0 degrees
        //Pose2d startPose = new Pose2d(62, 35, Math.toRadians(90));
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        TrajectorySequence deliverleft = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, -10))
                .addDisplacementMarker(() -> {
                    Hardware.dropServo.setPosition(0.43);
                    sleep(100);
                    Hardware.dropServo.setPosition(0.7);
                })
                .build();

        TrajectorySequence deliverRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, 10))
                .addDisplacementMarker(() -> {
                    Hardware.dropServo.setPosition(0.43);
                    sleep(100);
                    Hardware.dropServo.setPosition(0.7);
                })
                .build();

        TrajectorySequence deliverMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, 0))
                .addDisplacementMarker(() -> {
                    Hardware.dropServo.setPosition(0.43);
                    sleep(100);
                    Hardware.dropServo.setPosition(0.7);
                })



                .build();

        TrajectorySequence backup = drive.trajectorySequenceBuilder(deliverMiddle.end())
                .lineToConstantHeading(new Vector2d(-25, 0))

                .build();

        TrajectorySequence deliverBackdrop = drive.trajectorySequenceBuilder(backup.end())
                .lineToLinearHeading(new Pose2d(-30, -35, Math.toRadians(90)))
                .build();

        waitForStart();

        if (!isStopRequested()) {
        if(trajNumber == 1){
            drive.followTrajectorySequence(deliverleft);
        }
        else if(trajNumber == 2){
            drive.followTrajectorySequence(deliverMiddle);
        }
        else{
            drive.followTrajectorySequence(deliverRight);
        }

            drive.followTrajectorySequence(backup);
            drive.followTrajectorySequence(deliverBackdrop);
        }
    }
}