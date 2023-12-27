package org.firstinspires.ftc.teamcode.PowerPlayRobot;

import android.util.Pair;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TelemetrySubsystem extends SubsystemBase {
    private MultipleTelemetry multiTelemtry; // Driver Hub and FTC Dashboard
//    private List<Pair<String, BooleanSupplier> > monitors; // Pair<Caption, ValueSupplier>
    private BooleanSupplier monitor;

    public TelemetrySubsystem(Telemetry dhTelemetry, Telemetry dashTelemetry) {
        multiTelemtry = new MultipleTelemetry(dhTelemetry, dashTelemetry);
    }

    public void addMonitor(String caption, BooleanSupplier value) {
        monitor = value;
    }

    @Override
    public void periodic() {
//        for(Pair<String, BooleanSupplier> monitor : monitors) {
//            multiTelemtry.addData(monitor.first+":", monitor.second.getAsBoolean());
//        }

//        multiTelemtry.addData("Test:", monitor.getAsBoolean());
    }
}