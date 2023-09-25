package org.firstinspires.ftc.teamcode;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.inventors.ftc.robotbase.DriveConstants;
import org.inventors.ftc.robotbase.MecanumDrivePPV2;
import org.inventors.ftc.opencvpipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.AprilTagDetectionSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.RoadRunnerSubsystem;
import org.inventors.ftc.robotbase.GamepadExEx;

@Autonomous(name = "AutoOnlyParking", group = "Final Autonomous")
public class PowerPlayAutonomousParking extends CommandOpMode {

    PowerPlayRobot robot;

    protected DriveConstants RobotConstants;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    protected ElapsedTime runtime;
    protected MecanumDrivePPV2 drive;
    protected RoadRunnerSubsystem RR;
    protected AprilTagDetectionSubsystem april_tag;

    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        RobotConstants = new DriveConstants();

        robot = new PowerPlayRobot(hardwareMap, RobotConstants, telemetry, driverOp, toolOp, AUTO, true,
                false);

        drive = new MecanumDrivePPV2(hardwareMap, AUTO, RobotConstants);

        RR = new RoadRunnerSubsystem(drive, false);

        april_tag = new AprilTagDetectionSubsystem(robot.camera, telemetry);

        runtime = new ElapsedTime();
    }

    public void waitForStart() {
        /////////////////////////////////// Recognizing the Tag ///////////////////////////////////
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            april_tag.aprilTagCheck();
            sleep(20);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        runtime.reset();

        ///////////////////////////////// Running the Trajectories /////////////////////////////////

        int i = 0;

        if (isStopRequested()) return;

        RR.runHS();

        if (april_tag.getTagOfInterest().id == april_tag.LEFT) RR.runP1();
        else if (april_tag.getTagOfInterest().id == april_tag.RIGHT|| april_tag.getTagOfInterest() == null) RR.runP3();
        else RR.runTOMID();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}