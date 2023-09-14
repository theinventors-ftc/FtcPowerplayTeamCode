package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;


public class HeadingControllerSubsystem extends SubsystemBase {
    public enum Type {
        GYRO, CAMERA
    };

    private final Type fType;

    private DoubleSupplier gyroValue;
    private IntSupplier closestOrientationTarget;

    private double target = 0;

    private boolean enabled = false;
    private boolean findClosestTarget;

    private final double kP;
    private final double kI = 0;
    private final double kD = 0;

    PIDController controller;

    public HeadingControllerSubsystem(DoubleSupplier gyroValue,
                                      IntSupplier closestOrientationTarget) {
        kP = 0.02;
        controller = new PIDController(kP, kI, kD);
        this.gyroValue = gyroValue;
        this.closestOrientationTarget = closestOrientationTarget;
        fType = Type.GYRO;
    }

    public double calculateTurn() {
        double curValue = 0.0;
        if (fType == Type.CAMERA) {
//            curValue = camera.getPipeline().getElementsAnalogCoordinates()[0];
//            curValue = 0;
        } else {
            if (findClosestTarget) {
                target = closestOrientationTarget.getAsInt();
                findClosestTarget = false;
            }
            curValue = gyroValue.getAsDouble();
        }

        return controller.calculate(curValue);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setGyroTarget(double targetOrient) {
        double gyroValueDouble = gyroValue.getAsDouble();
        double dist, minDist;
        int minDistIdx, maxIdx;

        minDistIdx = 0;
        minDist = Math.abs(targetOrient - gyroValueDouble);
        maxIdx = (int) Math.ceil(Math.abs(gyroValueDouble) / 360);
        for (int i = -maxIdx; i <= maxIdx; i++) {
            dist = Math.abs(i * 360 + targetOrient - gyroValueDouble);
            if (dist < minDist) {
                minDistIdx = i;
                minDist = dist;
            }
        }

        target = minDistIdx * 360 + targetOrient;
        controller.setSetPoint(target);
    }

    public void toggleState() {
        enabled = !enabled;
        findClosestTarget = enabled || findClosestTarget;
    }
}