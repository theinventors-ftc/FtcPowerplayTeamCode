package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ClawSubsystem;

@Disabled
@Config
@TeleOp (name = "ServoLimitsTesting", group = "Tests")
public class ServoLimitsTesting extends LinearOpMode {
    ClawSubsystem claw;

    Telemetry dahsboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    public static double max = 1;
    public static double min = 0;

    double step = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new ClawSubsystem(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.dpad_up) claw.release();
            if(gamepad1.dpad_down) claw.grab();

            dahsboardTelemetry.addData("Smth", "");
        }
    }
}
