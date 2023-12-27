package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.CommandBase;

import org.inventors.ftc.robotbase.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem drivetrain;
    private final DoubleSupplier forwardSpeed, strafeSpeed, turnSpeed;

    public DriveCommand(DriveSubsystem drivetrain, DoubleSupplier forwardSpeed,
                               DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed) {
        this.drivetrain = drivetrain;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.turnSpeed = turnSpeed;
        addRequirements(this.drivetrain);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void execute() {
        drivetrain.drive(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble());
    }
}
