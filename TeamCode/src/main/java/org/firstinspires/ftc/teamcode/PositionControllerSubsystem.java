package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import java.awt.font.NumericShaper;
import java.util.function.DoubleSupplier;

public class PositionControllerSubsystem extends SubsystemBase {
    public enum Type {
        APRILTAG
    }

    private final Type type;
    private DoubleSupplier errorX;
    private DoubleSupplier errorY;
    private DoubleSupplier errorRX;

    private PIDFControllerEx controllerX;
    private PIDFControllerEx controllerY;
    private PIDFControllerEx controllerRX;

    private double KpX, KiX, KdX;
    private double KpY, KiY, KdY;
    private double KpRX, KiRX, KdRX;

    private double alpha;

    public PositionControllerSubsystem(DoubleSupplier errorX, DoubleSupplier errorY, DoubleSupplier errorRX, double KpX, double KpY, double KpRX,  double KiX, double KiY, double KiRX, double KdX, double KdY, double KdRX, double alpha, double integralWorkingBounds, double integralClippingBounds) {
        type = Type.APRILTAG;
        this.errorX = errorX;
        this.errorY = errorY;
        this.errorRX = errorRX;

        this.KpX = KpX;
        this.KpY = KpY;
        this.KpRX = KpRX;
        this.KdX = KdX;
        this.KdY = KdY;
        this.KdRX = KdRX;
        this.controllerX = new PIDFControllerEx(KpX, KiX, KdX, 0, alpha, integralWorkingBounds, integralClippingBounds);
        this.controllerY = new PIDFControllerEx(KpY, KiY, KdY, 0, alpha, integralWorkingBounds, integralClippingBounds);
        this.controllerRX = new PIDFControllerEx(KpRX, KiRX, KdRX, 0, alpha, integralWorkingBounds, integralClippingBounds);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double calculateStrafeSpeed() {
        return Range.clip(controllerX.calculate(errorX.getAsDouble()), -0.4, 0.4);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double calculateForwardSpeed() {
        return Range.clip(controllerY.calculate(errorY.getAsDouble()), -0.4, 0.4);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double calculateRotateSpeed() {
        return Range.clip(controllerRX.calculate(errorRX.getAsDouble()), -0.25, 0.25);
    }
}
