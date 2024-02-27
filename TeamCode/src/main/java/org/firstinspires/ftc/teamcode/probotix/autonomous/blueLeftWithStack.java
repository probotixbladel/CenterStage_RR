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

@Autonomous(name="blueLeftWithStack", group="probotix")
public class blueLeftWithStack extends LinearOpMode {
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
                .lineToConstantHeading(new Vector2d(-30, -11))
                .build();

        TrajectorySequence backupLeft = drive.trajectorySequenceBuilder(deliverleft.end())
                .lineToConstantHeading(new Vector2d(-15, -11))
                .build();

        TrajectorySequence deliverBackdropLeft = drive.trajectorySequenceBuilder(backupLeft.end())
                .lineToLinearHeading(new Pose2d(-18, -40, Math.toRadians(90)))
                .build();

        TrajectorySequence moveCenterLeft = drive.trajectorySequenceBuilder(deliverBackdropLeft.end())
                .lineToLinearHeading(new Pose2d(-49,-38, Math.toRadians(91)))
                .build();
        TrajectorySequence driveToStackLeft = drive.trajectorySequenceBuilder(moveCenterLeft.end())
                .lineToLinearHeading(new Pose2d(-49,55,Math.toRadians(91)))
                .build();




        TrajectorySequence slowlyLeft = drive.trajectorySequenceBuilder(driveToStackLeft.end())
                .lineToLinearHeading(new Pose2d(-49,65,Math.toRadians(90)))
                .build();

        TrajectorySequence slowlyBackLeft = drive.trajectorySequenceBuilder(slowlyLeft.end())
                .lineToLinearHeading(new Pose2d(-49,50,Math.toRadians(90)))
                .build();
        TrajectorySequence goBackToBackboardLeft = drive.trajectorySequenceBuilder(slowlyBackLeft.end())
                .lineToLinearHeading(new Pose2d(-49,-30,Math.toRadians(90)))
                .build();
        TrajectorySequence deliverStackLeft = drive.trajectorySequenceBuilder(goBackToBackboardLeft.end())
                .lineToLinearHeading(new Pose2d(-33,-40,Math.toRadians(90)))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(deliverStackLeft.end())
                .lineToConstantHeading(new Vector2d(-38, -45))
                .build();

        //******************************** LEFT ******************************//

        //******************************** MIDDLE ****************************//

        TrajectorySequence deliverMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-31, 0))
                .build();

        TrajectorySequence backupMiddle = drive.trajectorySequenceBuilder(deliverMiddle.end())
                .lineToConstantHeading(new Vector2d(-26,0))
                .build();

        TrajectorySequence deliverBackdropMiddle = drive.trajectorySequenceBuilder(backupMiddle.end())
                .lineToLinearHeading(new Pose2d(-28,-40,Math.toRadians(90)))
                .build();

        TrajectorySequence moveCenterMiddle = drive.trajectorySequenceBuilder(deliverBackdropMiddle.end())
                .lineToLinearHeading(new Pose2d(-53.5,-41, Math.toRadians(90)))
                .build();
        TrajectorySequence driveToStackMiddle = drive.trajectorySequenceBuilder(moveCenterMiddle.end())
                .splineToLinearHeading(new Pose2d(-55,57),Math.toRadians(90))
                .build();

        TrajectorySequence slowlyMiddle = drive.trajectorySequenceBuilder(driveToStackMiddle.end())
                .lineToLinearHeading(new Pose2d(-55,66,Math.toRadians(90)))
                .build();

        TrajectorySequence slowlyBackMiddle = drive.trajectorySequenceBuilder(slowlyMiddle.end())
                .lineToLinearHeading(new Pose2d(-55,50,Math.toRadians(90)))
                .build();
        TrajectorySequence goBackToBackboardMiddle = drive.trajectorySequenceBuilder(slowlyBackMiddle.end())
                .lineToLinearHeading(new Pose2d(-53.5,-30,Math.toRadians(90)))
                .build();
        TrajectorySequence deliverStackMiddle = drive.trajectorySequenceBuilder(goBackToBackboardMiddle.end())
                .lineToLinearHeading(new Pose2d(-22,-43,Math.toRadians(90)))
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(deliverStackMiddle.end())
                .lineToConstantHeading(new Vector2d(-50, -37))
                .build();

        //******************************** MIDDLE ****************************//

        //**************************** RIGHT *************************************//

        TrajectorySequence deliverRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-28,0))
                .build();

        TrajectorySequence backupRight = drive.trajectorySequenceBuilder(deliverRight.end())
                .lineToLinearHeading(new Pose2d(-29,5, Math.toRadians(-90)))
                .build();

        TrajectorySequence deliverBackdropRight = drive.trajectorySequenceBuilder(backupRight.end())
                .lineToLinearHeading(new Pose2d(-34, -40, Math.toRadians(90)))
                .build();

        TrajectorySequence moveCenterRight = drive.trajectorySequenceBuilder(deliverBackdropRight.end())
                .lineToLinearHeading(new Pose2d(-54,-41, Math.toRadians(90)))
                .build();
        TrajectorySequence driveToStackRight = drive.trajectorySequenceBuilder(moveCenterRight.end())
                .lineToLinearHeading(new Pose2d(-54,57,Math.toRadians(90)))
                .build();

        TrajectorySequence slowlyRight = drive.trajectorySequenceBuilder(driveToStackRight.end())
                .lineToLinearHeading(new Pose2d(-54,67,Math.toRadians(90)))
                .build();

        TrajectorySequence slowlyBackRight = drive.trajectorySequenceBuilder(slowlyRight.end())
                .lineToLinearHeading(new Pose2d(-54,50,Math.toRadians(90)))
                .build();
        TrajectorySequence goBackToBackboardRight = drive.trajectorySequenceBuilder(slowlyBackRight.end())
                .lineToLinearHeading(new Pose2d(-54,-30,Math.toRadians(90)))
                .build();
        TrajectorySequence deliverStackRight = drive.trajectorySequenceBuilder(goBackToBackboardRight.end())
                .lineToLinearHeading(new Pose2d(-28,-41,Math.toRadians(90)))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(deliverStackRight.end())
                .lineToConstantHeading(new Vector2d(-50, -37))
                .build();

        //**************************** RIGHT *************************************//


        waitForStart();
        webcam.stopStreaming();
        if (!isStopRequested()) {
        if(trajNumber == 1){

            Hardware.armMotor.setPower(1);
            Hardware.liftMotor.setPower(1);
            Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
            sleep(10);
            drive.followTrajectorySequence(deliverleft);
            Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
            sleep(500);
            drive.followTrajectorySequence(backupLeft);
            Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
            drive.followTrajectorySequence(deliverBackdropLeft);
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
            //******************** pickup 2 pixels stack ***********************//

            drive.followTrajectorySequence(moveCenterLeft);
            Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+50);
            Hardware.armMotor.setTargetPosition(1650);
            Hardware.flipServo.setPosition(DriveConstants.flipServoPickUp);
            Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
            sleep(500);
            drive.followTrajectorySequence(driveToStackLeft);
            Hardware.liftMotor.setTargetPosition(380);
            sleep(50);
            drive.followTrajectorySequence(slowlyLeft);
            sleep(100);
            Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
            sleep(100);
            Hardware.armMotor.setTargetPosition(1550);
            sleep(500);
            drive.followTrajectorySequence(slowlyBackLeft);
            drive.followTrajectorySequence(goBackToBackboardLeft);
            drive.followTrajectorySequence(deliverStackLeft);
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
            //******************** pickup 2 pixels stack ***********************//
            drive.followTrajectorySequence(parkLeft);

        }
        else if(trajNumber == 2){

            Hardware.armMotor.setPower(1);
            Hardware.liftMotor.setPower(1);
            Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
            sleep(10);
            drive.followTrajectorySequence(deliverMiddle);
            Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
            sleep(500);
            drive.followTrajectorySequence(backupMiddle);
            Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
            sleep(500);
            drive.followTrajectorySequence(deliverBackdropMiddle);
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


            drive.followTrajectorySequence(moveCenterMiddle);
            Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+50);
            Hardware.armMotor.setTargetPosition(1650);
            Hardware.flipServo.setPosition(DriveConstants.flipServoPickUp);
            Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
            sleep(500);
            drive.followTrajectorySequence(driveToStackMiddle);
            Hardware.liftMotor.setTargetPosition(380);
            sleep(50);
            drive.followTrajectorySequence(slowlyMiddle);
            sleep(100);
            Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
            sleep(100);
            Hardware.armMotor.setTargetPosition(1550);
            sleep(500);
            drive.followTrajectorySequence(slowlyBackMiddle);
            drive.followTrajectorySequence(goBackToBackboardMiddle);
            drive.followTrajectorySequence(deliverStackMiddle);
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


            drive.followTrajectorySequence(parkMiddle);

        }
        else{

            Hardware.armMotor.setPower(1);
            Hardware.liftMotor.setPower(1);
            Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
            sleep(10);
            drive.followTrajectorySequence(deliverRight);
            drive.followTrajectorySequence(backupRight);
            Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
            sleep(500);
            drive.followTrajectorySequence(deliverBackdropRight);
            Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
            Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
            Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
            Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
            sleep(700);
            Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
            sleep(700);
            Hardware.armMotor.setTargetPosition(0);
            sleep(100);
            Hardware.liftMotor.setTargetPosition(10);
            Hardware.flipServo.setPosition(DriveConstants.flipServoInit);
            sleep(200);


            drive.followTrajectorySequence(moveCenterRight);
            Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+50);
            Hardware.armMotor.setTargetPosition(1650);
            Hardware.flipServo.setPosition(DriveConstants.flipServoPickUp);
            Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
            sleep(500);
            drive.followTrajectorySequence(driveToStackRight);
            Hardware.liftMotor.setTargetPosition(380);
sleep(50);
            drive.followTrajectorySequence(slowlyRight);
            sleep(100);
            Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
            sleep(100);
            Hardware.armMotor.setTargetPosition(1550);
            sleep(500);
            drive.followTrajectorySequence(slowlyBackRight);
            drive.followTrajectorySequence(goBackToBackboardRight);
            drive.followTrajectorySequence(deliverStackRight);
            Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
            Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
            Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
            sleep(700);
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
