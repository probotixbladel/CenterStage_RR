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
@Autonomous(name="Left", group="probotix")
public class Left extends LinearOpMode{
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

        Trajectory delConeFW2 = drive.trajectoryBuilder(delConeRS.end().plus(new Pose2d(0,0, Math.toRadians(-43))))
                .forward(-9,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/3,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory delConeFW3 = drive.trajectoryBuilder(delConeFW2.end())
                .forward(9)
                .build();



        /*Trajectory delConeBW = drive.trajectoryBuilder(delConeFW2.end())
                .splineTo(new Vector2d(-5, 0), Math.toRadians(0))
                .build();
*/
        Trajectory park1 = drive.trajectoryBuilder(delConeFW3.end().plus(new Pose2d(0,0, Math.toRadians(43))))
                .lineTo(new Vector2d(-51,-27))
                .build();

        Trajectory park2 = drive.trajectoryBuilder(delConeFW2.end())
                .lineTo(new Vector2d(-51, 0))
                .build();

        Trajectory park3 = drive.trajectoryBuilder(delConeFW3.end().plus(new Pose2d(0,0, Math.toRadians(43))))
                .lineTo(new Vector2d(-51, 25))
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
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    }}}}}}