package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlayRobot.PowerPlayRobot;
import org.inventors.ftc.robotbase.DriveConstants;
import org.inventors.ftc.robotbase.GamepadExEx;

@TeleOp(name = "PowerPlayBucharest2023", group = "Final TeleOPs")
public class PowerPlayTeleOp extends CommandOpMode {
    private PowerPlayRobot robot;

    private DriveConstants RobotConstants;

    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        RobotConstants = new DriveConstants();

        RobotConstants.COMMON_FEED_FORWARD = true;

        robot = new PowerPlayRobot(hardwareMap, RobotConstants, telemetry, driverOp, toolOp);
    }
}