package org.firstinspires.ftc.teamcode.PowerPlayRobot.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;

public class CommandAction implements Action {
    private Command command;
    private boolean isFinished;

    public CommandAction(Command command){
        this.command = command;
    }

    public void initialize() {
        command.initialize();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        command.execute();
        return command.isFinished();
    }
}
