package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SliderSubsystem extends SubsystemBase {
    private final MotorEx motor;

    public enum Level {
        PARK, INTAKE, ONE, TWO, THREE
    }

    private Level level;
    private final int[] levelPositions = {0, 200, 533, 1121, 1678};

    public SliderSubsystem(HardwareMap hardwareMap) {
        this.motor = new MotorEx(hardwareMap, "slider");
        this.motor.setInverted(true);
        this.motor.setRunMode(MotorEx.RunMode.PositionControl);
        this.motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setPositionTolerance(10);   // allowed maximum error
        level = Level.PARK;
    }

    public void setLevel(Level newLevel) {
        level = newLevel;
        int levelIdx = 1;
        if (level == Level.INTAKE)
            levelIdx = 0;
        else if (level == Level.ONE)
            levelIdx = 2;
        else if (level == Level.TWO)
            levelIdx = 3;
        else if (level == Level.THREE)
            levelIdx = 4;
        motor.setTargetPosition(levelPositions[levelIdx]);
    }

    public void run() {
        motor.set(0.5);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean atTargetLevel() {
        return motor.atTargetPosition();
    }
}
