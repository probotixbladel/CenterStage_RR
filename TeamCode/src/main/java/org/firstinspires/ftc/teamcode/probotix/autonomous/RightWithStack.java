package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.examples.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name="RightWithStack", group="probotix")
public class RightWithStack extends LinearOpMode{
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
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        Trajectory delConeFW = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-65,0))
                .build();


        Trajectory delConeRS = drive.trajectoryBuilder(delConeFW.end())
                .lineTo(new Vector2d(-51,0))
                .build();

        Trajectory delConeFW2 = drive.trajectoryBuilder(delConeRS.end().plus(new Pose2d(0,0, Math.toRadians(45))))
                .forward(-10,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/3,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory delConeFW3 = drive.trajectoryBuilder(delConeFW2.end())
                .forward(10)
                .build();

        Trajectory moveToStack = drive.trajectoryBuilder(delConeFW3.end().plus(new Pose2d(0,0, Math.toRadians(-45))))
                .lineToLinearHeading(new Pose2d(-51,15,Math.toRadians(-86)))
                .build();

        Trajectory moveToStack2 = drive.trajectoryBuilder(moveToStack.end())
                .lineToConstantHeading(new Vector2d(-53,24.1))
                .build();

        Trajectory moveAwayFromStack = drive.trajectoryBuilder(moveToStack2.end())
                .forward(15)
                .build();

        Trajectory deliverStackCone1 = drive.trajectoryBuilder(moveAwayFromStack.end())
                .lineToLinearHeading(new Pose2d(-60.9,-6.9,Math.toRadians(45)))
                .build();

        Trajectory park2 = drive.trajectoryBuilder(deliverStackCone1.end())
                .lineToLinearHeading(new Pose2d(-51,0,Math.toRadians(0)))
                .build();





        /*Trajectory delConeBW = drive.trajectoryBuilder(delConeFW2.end())
                .splineTo(new Vector2d(-5, 0), Math.toRadians(0))
                .build();

*/
        /*
        Trajectory park1 = drive.trajectoryBuilder(delConeFW3.end().plus(new Pose2d(0,0, Math.toRadians(-45))))
                .lineTo(new Vector2d(-51,-27))
                .build();

        Trajectory park2 = drive.trajectoryBuilder(delConeFW2.end())
                .lineTo(new Vector2d(-51, 0))
                .build();

        Trajectory park3 = drive.trajectoryBuilder(delConeFW3.end().plus(new Pose2d(0,0, Math.toRadians(-45))))
                .lineTo(new Vector2d(-51, 25))
                .build();
                */


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
            Hardware.liftMotor.setTargetPosition(-900);
            Hardware.liftMotor.setPower(0.6);
            drive.followTrajectory(delConeFW);
            drive.followTrajectory(delConeRS);
            drive.turn(Math.toRadians(45));
            Hardware.liftMotor.setTargetPosition(-4140);
            Hardware.liftMotor.setPower(0.6);
            sleep(2000);
            drive.followTrajectory(delConeFW2);
            sleep(1000);
            Hardware.grabServo.setPosition(0.70);
            sleep(500);
            Hardware.grabServo.setPosition(0.15);//hi

            sleep(100);
            drive.followTrajectory((delConeFW3));
            //Hardware.grabServo.setPosition(0.70);
            Hardware.liftMotor.setTargetPosition(0);
            Hardware.liftMotor.setPower(0.6);
            sleep(2000);


            drive.turn(Math.toRadians(-45));
            Hardware.grabServo.setPosition(0.70);
            Hardware.liftMotor.setTargetPosition(-1700);
            Hardware.liftMotor.setPower(0.6);
            drive.followTrajectory(moveToStack);
            drive.followTrajectory(moveToStack2);
            Hardware.liftMotor.setTargetPosition(-700);
            Hardware.liftMotor.setPower(0.6);
            sleep(1000);
            Hardware.grabServo.setPosition(0);
            sleep(1000);
            Hardware.liftMotor.setTargetPosition(-1700);
            Hardware.liftMotor.setPower(0.6);
            sleep(1000);
            Hardware.liftMotor.setTargetPosition(-4140);
            Hardware.liftMotor.setPower(0.8);
            drive.followTrajectory(moveAwayFromStack);
            drive.followTrajectory(deliverStackCone1);
            sleep(1000);
            Hardware.grabServo.setPosition(0.70);
            sleep(500);
            drive.followTrajectory(park2);
            Hardware.liftMotor.setTargetPosition(0);
            Hardware.liftMotor.setPower(0.8);





            sleep(2000);


            //drive.followTrajectory(delConeFW2);

            /*drive.followTrajectory(delConeFW2);
*/
            /*
            if (lastId == 0) {
                drive.followTrajectory(park1);
            } else if (lastId == 2) {
                //drive.followTrajectory(park2);
            } else if (lastId == 4) {
                drive.followTrajectory(park3);
            }

             */


        }
    }
}
