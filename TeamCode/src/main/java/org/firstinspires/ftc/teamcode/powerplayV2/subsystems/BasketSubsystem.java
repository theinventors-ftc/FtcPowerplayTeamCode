package org.firstinspires.ftc.teamcode.powerplayV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BasketSubsystem extends SubsystemBase {
    private ServoImplEx servo;
    private final double MIN = 0.01, MAX = 0.48;

    enum State {
        OUTTAKE,
        TRAVEL
    }

    State state;

    public BasketSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "basket");
        state = State.TRAVEL;
    }

    public void setTravel() {
        servo.setPosition(MIN);
        state = State.TRAVEL;
    }

    public void setOuttake() {
        servo.setPosition(MAX);
        state = State.OUTTAKE;
    }

    public State getState() {
        return this.state;
    }
}