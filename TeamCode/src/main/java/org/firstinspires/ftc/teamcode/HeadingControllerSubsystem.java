package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.function.DoubleSupplier;

public class HeadingControllerSubsystem extends SubsystemBase {
    private DoubleSupplier taglineRot;
    PIDController controller;
    private final double kP = 0.06;
    private final double kI = 0;
    private final double kD = 0;

    public HeadingControllerSubsystem(DoubleSupplier taglineRot) {
        controller = new PIDController(kP, kI, kD);
        this.taglineRot = taglineRot;
    }

    public double calculateTurn() {
        return controller.calculate(taglineRot.getAsDouble());
    }
}
