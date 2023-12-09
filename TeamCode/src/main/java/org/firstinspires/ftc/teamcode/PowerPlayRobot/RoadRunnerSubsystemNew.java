package org.firstinspires.ftc.teamcode.PowerPlayRobot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.PowerPlayRobot.commands.CommandAction;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ArmSubsystem;
import org.inventors.ftc.robotbase.drivebase.MecanumDriveSubsystem;

public class RoadRunnerSubsystemNew extends SubsystemBase {

    protected MecanumDriveSubsystem driveRR;
    protected Action HomeToScoring;
    protected Action Parking_1;
    protected Action Parking_2;
    protected Action Parking_3;
    protected Action Test;

    protected ArmSubsystem arm;
    protected InstantCommand armCommand;
    protected CommandAction CommandToAction;

    protected Pose2d homePose = new Pose2d(35  ,-72 + 4.4 + (13.2/2), Math.toRadians(90));
    protected double midPoseY = -20;
    protected Pose2d midPose = new Pose2d(35, -20, Math.toRadians(-13.25));
    protected Pose2d scoringPose = new Pose2d(24 + 21.80769231, 0 - 5.538461538, Math.toRadians(90));

    protected Pose2d parking_1 = new Pose2d(12, -12, Math.toRadians(90));
    protected Pose2d parking_3 = new Pose2d(60, -12, Math.toRadians(90));

    public RoadRunnerSubsystemNew(MecanumDriveSubsystem drive){
        this.driveRR = drive;

        /////////////////////////////////INIT////////////////////////////
        arm = new ArmSubsystem(hardwareMap);
        armCommand = new InstantCommand(arm::setIntake, arm);
        //////////////////////////////////////////////////////////////////

        Test = driveRR.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(20)
                .build();

        HomeToScoring = driveRR.actionBuilder(homePose)
                .lineToXSplineHeading(midPoseY, Math.toRadians(-13.25))
                .splineToConstantHeading(scoringPose.component1(), Math.PI / 2)
                .build();

        Parking_1 = driveRR.actionBuilder(scoringPose)
                .splineToSplineHeading(parking_1, Math.toRadians(-90))
                .build();

        Parking_2 = driveRR.actionBuilder(scoringPose)
                .splineToSplineHeading(midPose, Math.toRadians(-90))
                .build();

        Parking_3 = driveRR.actionBuilder(scoringPose)
                .splineToSplineHeading(parking_3, Math.toRadians(-90))
                .build();

    }

    public void runHOME_TO_SCORING(){Actions.runBlocking(HomeToScoring);}

    public void runPARKING_1(){Actions.runBlocking(Parking_1);}

    public void runPARKING_2(){Actions.runBlocking(Parking_2);}

    public void runPARKING_3(){Actions.runBlocking(Parking_3);}

    public void TEST(){Actions.runBlocking(new ParallelAction(Test, new CommandAction(armCommand)));}
}

