package org.firstinspires.ftc.teamcode.tests;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.DriveConstants;
import org.inventors.ftc.robotbase.MecanumDrivePPV2;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.AprilTagDetectionSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.BasketSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ConeDetectorSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.FrontSliderSubsystem;
import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.LimitSwitchSubsystem;
import org.inventors.ftc.robotbase.GamepadExEx;

@Disabled
@Autonomous(name = "TestAutonomous", group = "Tests")
public class CommandGroupTest extends CommandOpMode {
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
    protected FrontSliderSubsystem frontSlider;
    protected LimitSwitchSubsystem rightSwitch, leftSwitch;
    protected ConeDetectorSubsystem cone_detector;
    protected boolean april_tag_found = false;
    Telemetry dashboardTelemetry;
    protected SequentialCommandGroup scoringCommand, test;
    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        RobotConstants = new DriveConstants();

        robot = new PowerPlayRobot(hardwareMap, RobotConstants, telemetry, driverOp, toolOp, AUTO, true,
                false);

        drive = new MecanumDrivePPV2(hardwareMap, AUTO, RobotConstants);

        RR = new RoadRunnerSubsystem(drive, false);

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        basket =  new BasketSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        rightSwitch = new LimitSwitchSubsystem(hardwareMap, "rightSwitch");
        leftSwitch = new LimitSwitchSubsystem(hardwareMap, "leftSwitch");
        frontSlider = new FrontSliderSubsystem(hardwareMap, () -> rightSwitch.getState(),
                () -> leftSwitch.getState());

        scoringCommand = new SequentialCommandGroup(
                new InstantCommand(arm::setMid, arm),
                new WaitCommand(300),
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.AUTO_SCORING),
                new InstantCommand(basket::setOuttake, basket),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        new InstantCommand(basket::setTravel, basket),
                        new InstantCommand(arm::setMid, arm)
                ),
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW)
        );
        runtime = new ElapsedTime();
    }

    public void waitForStart() {
        /////////////////////////////////// Recognizing the Tag ////////////////////////////////////
        while (!isStarted() && !isStopRequested()) {
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

        test = new SequentialCommandGroup(
                new InstantCommand(arm::setIntake, arm),
                new WaitCommand(1500),
                new InstantCommand(arm::setTravel, arm),
                scoringCommand,
                new WaitCommand(1500),
                new InstantCommand(arm::setIntake, arm)
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