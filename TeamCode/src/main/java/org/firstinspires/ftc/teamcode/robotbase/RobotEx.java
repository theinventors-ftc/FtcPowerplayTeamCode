package org.firstinspires.ftc.teamcode.robotbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RobotEx {
    protected final Telemetry telemetry;
    protected final FtcDashboard dashboard;
    protected final Telemetry dashboardTelemetry;

    protected final GamepadEx driverOp;
    protected final GamepadEx toolOp;

    protected final MecanumDriveSubsystem drive;
    protected final MecanumDriveCommand driveCommand;

    protected final IMUSubsystem gyro;
    protected final Camera camera;

    protected final HeadingControllerSubsystem gyroFollow;
    protected final HeadingControllerSubsystem cameraFollow;

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                   GamepadEx toolOp) {
        this(hardwareMap, telemetry, driverOp, toolOp, true);
    }

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                   GamepadEx toolOp, Boolean useCameraFollower) {
        ///////////////////////////////////////// Gamepads /////////////////////////////////////////
        this.driverOp = driverOp;
        this.toolOp = toolOp;

        /////////////////////////////////////// FTC Dashboard //////////////////////////////////////
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        this.telemetry = telemetry;

        //////////////////////////////////////////// IMU ///////////////////////////////////////////
        gyro = new IMUSubsystem(hardwareMap, this.telemetry, dashboardTelemetry);
        CommandScheduler.getInstance().registerSubsystem(gyro);

        ////////////////////////////////////////// Camera //////////////////////////////////////////
        camera = new Camera(hardwareMap, dashboard, telemetry,
                () -> this.driverOp.getButton(GamepadKeys.Button.BACK));

        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
        drive = new MecanumDriveSubsystem(hardwareMap);
        driveCommand = new MecanumDriveCommand(drive, driverOp::getLeftX, driverOp::getLeftY,
                this::drivetrainTurn, gyro::getRawValue,
                () -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        CommandScheduler.getInstance().registerSubsystem(drive);
        drive.setDefaultCommand(driveCommand);

        /////////////////////////////////////// Gyro Follower //////////////////////////////////////
        gyroFollow = new HeadingControllerSubsystem(gyro::getValue,
                gyro::findClosestOrientationTarget);
        new Trigger(() -> driverOp.getRightY() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(0), gyroFollow));
        new Trigger(() -> driverOp.getRightY() <= 0.2).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(180), gyroFollow));
        new Trigger(() -> driverOp.getRightX() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(90), gyroFollow));
        new Trigger(() -> driverOp.getRightX() <= 0.2).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(-90), gyroFollow));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(0)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(180)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(-90)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(90)));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(gyroFollow::toggleState, gyroFollow));

        ////////////////////////////////////// Camera Follower /////////////////////////////////////
        cameraFollow = new HeadingControllerSubsystem(camera);
        if (useCameraFollower)
            driverOp.getGamepadButton(GamepadKeys.Button.START)
                    .whenPressed(new InstantCommand(cameraFollow::toggleState, cameraFollow));

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
        initMechanisms(hardwareMap);
    }

    public void initMechanisms(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

    public double drivetrainTurn() {
        if (gyroFollow.isEnabled())
            return gyroFollow.calculateTurn();
        if (cameraFollow.isEnabled())
            return cameraFollow.calculateTurn();
        return driverOp.getRightX();
    }

    public void telemetryUpdate() {
        telemetry.update();
    }

    public void dashboardTelemetryUpdate() {
        dashboardTelemetry.update();
    }
}