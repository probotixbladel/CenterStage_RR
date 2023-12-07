package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(name="BlueRight", group="probotix")
public class BlueRight extends LinearOpMode {
    private hardware Hardware;

    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //forward -x, backward +x, left -y, right +y
        // We want to start the bot at x: 0, y: 0, heading: 0 degrees
        //Pose2d startPose = new Pose2d(62, 35, Math.toRadians(90));
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence deliverMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30,0))
                        .addDisplacementMarker(()->{
                            Hardware.dropServo.setPosition(0.43);
                            sleep(100);
                            Hardware.dropServo.setPosition(0.7);
                        })
                .build();

        TrajectorySequence backToBeginning = drive.trajectorySequenceBuilder(deliverMiddle.end())
                .lineToConstantHeading(new Vector2d(-5,0))
                .addTemporalMarker(1,()->{
                    Hardware.armMotor.setTargetPosition(-200);
                    Hardware.armMotor.setPower(0.8);
                })
                .build();

        TrajectorySequence Turn = drive.trajectorySequenceBuilder(backToBeginning.end())
                .turn(Math.toRadians(90))
                        .build();



        TrajectorySequence toBackboardY = drive.trajectorySequenceBuilder(Turn.end())
                .lineToConstantHeading(new Vector2d(-5,-85))
                .build();

        TrajectorySequence toBackboardX = drive.trajectorySequenceBuilder(toBackboardY.end())
                        .strafeTo(new Vector2d(-30,-85))
                .addTemporalMarker(1,()->{
                    Hardware.armMotor.setTargetPosition(0);

                })
                .build();



        waitForStart();

        if (!isStopRequested()) {

            drive.followTrajectorySequence(deliverMiddle);
            drive.followTrajectorySequence(backToBeginning);
            drive.followTrajectorySequence(Turn);

            //Hardware.armMotor.setTargetPosition(-500);
            //drive.turn(Math.toRadians(90));
            drive.followTrajectorySequence(toBackboardY);
            drive.followTrajectorySequence(toBackboardX);
        }
    }
}