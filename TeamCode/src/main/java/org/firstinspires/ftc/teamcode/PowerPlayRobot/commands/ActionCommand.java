package org.firstinspires.ftc.teamcode.PowerPlayRobot.commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PowerPlayRobot.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ActionCommand extends CommandBase {
    private final Action action;
    private TelemetryPacket packet;
    private Boolean isFinished;

    public ActionCommand(Action action, TelemetryPacket packet){
        this.action = action;
        this.packet = packet;
    }

    @Override
    public void execute() {
        isFinished = action.run(packet);
    }

    @Override
    public void end(boolean interrupted) {
        // we should check what rr does in the end somewhere. Maybe nothing?
   }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
