package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@Autonomous(name="autonoomtest2", group="probotix")
public class autonoomtest2 extends LinearOpMode {


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        //was x=10 y=-8 rad 90
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                //.splineTo(new Vector2d(0, -5), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);

    }
}