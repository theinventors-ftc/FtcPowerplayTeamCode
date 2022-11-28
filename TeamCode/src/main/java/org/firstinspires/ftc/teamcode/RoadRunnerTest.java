package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RoadRunnerTest", group = "RoadRunner")
public class RoadRunnerTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Trajectory t1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(20)
                    .build();
            Trajectory t2 = drive.trajectoryBuilder(t1.end())
                    .lineToSplineHeading(new Pose2d(70,30, Math.toRadians(90)))
                    .build();
            Trajectory t3 = drive.trajectoryBuilder(t2.end())
                    .lineToSplineHeading(new Pose2d(0,0, Math.toRadians(0)))
                    .build();

            waitForStart();

            if(isStopRequested()) return;

            drive.followTrajectory(t1);
            drive.followTrajectory(t2);
            drive.followTrajectory(t3);
        }
}


