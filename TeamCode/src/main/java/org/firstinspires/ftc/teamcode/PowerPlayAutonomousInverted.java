package org.firstinspires.ftc.teamcode;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.inventors.ftc.robotbase.DriveConstants;
import org.inventors.ftc.robotbase.MecanumDrivePPV2;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.AprilTagDetectionSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.BasketSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ElevatorSubsystem;
import org.inventors.ftc.robotbase.GamepadExEx;

import java.util.HashMap;

@Autonomous(name = "AutoOneConeInv", group = "Final Autonomous")
public class PowerPlayAutonomousInverted extends CommandOpMode {
    PowerPlayRobot robot;

    protected DriveConstants RobotConstants;

    protected ElapsedTime runtime;
    protected MecanumDrivePPV2 drive;
    protected RoadRunnerSubsystem RR;
    protected AprilTagDetectionSubsystem april_tag;
    protected ClawSubsystem claw;
    protected ElevatorSubsystem elevator;
    protected BasketSubsystem basket;
    protected ArmSubsystem arm;
    protected boolean april_tag_found = false;
    protected SequentialCommandGroup scoringCommand;
    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        RobotConstants = new DriveConstants();

        robot = new PowerPlayRobot(hardwareMap, RobotConstants, telemetry, driverOp, toolOp, AUTO, true,
                false);

        drive = new MecanumDrivePPV2(hardwareMap, AUTO, RobotConstants);

        RR = new RoadRunnerSubsystem(drive, true);

        april_tag = new AprilTagDetectionSubsystem(robot.camera, telemetry);

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        basket =  new BasketSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);

        scoringCommand = new SequentialCommandGroup(
                new InstantCommand(arm::setMid, arm),
                new WaitCommand(300),
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.HIGH),
                new InstantCommand(basket::setOuttake, basket),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        new InstantCommand(basket::setTravel, basket),
                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW)
                ));

        runtime = new ElapsedTime();
    }
    public void waitForStart() {
        /////////////////////////////////// Recognizing the Tag ///////////////////////////////////
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

        if (isStopRequested()) return;

        schedule(new SequentialCommandGroup(
                new InstantCommand(RR::runHS2, RR),
                scoringCommand
        ));

        //Select Command Auto
        new Trigger(() -> runtime.seconds() >= 20).whenActive(
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(april_tag.LEFT, new InstantCommand(RR::runP1, RR));
                            put(april_tag.MIDDLE, new InstantCommand(RR::runTOMID, RR));
                            put(april_tag.RIGHT, new InstantCommand(RR::runP3, RR));
                            put(-1, new InstantCommand(RR::runTOMID, RR));
                        }},
                        () -> april_tag_found ? april_tag.getTagOfInterest().id : -1
                )
        );

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
            run();
        }
        reset();
    }
}