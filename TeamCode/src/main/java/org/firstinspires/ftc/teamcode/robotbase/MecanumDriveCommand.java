package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drivetrain;
    private final DoubleSupplier forward, strafe, turn;
    private final DoubleSupplier heading;
    private final DoubleSupplier speed;

    public MecanumDriveCommand(MecanumDriveSubsystem drive, DoubleSupplier forward,
                               DoubleSupplier strafe, DoubleSupplier turn, DoubleSupplier heading,
                               DoubleSupplier speed) {
        this.drivetrain = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.heading = heading;
        this.speed = speed;
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(-forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble(),
                heading.getAsDouble(), speed.getAsDouble());
    }
}
