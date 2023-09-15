package org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class FrontSliderSubsystem extends SubsystemBase {
    private final CRServoImplEx rightServo, leftServo;
    private BooleanSupplier rightSup, leftSup;

    private double MAX_POWER = 0.7;
    private double AUTO_MAX_PWR = 0.8;

    public enum State {
        OPENING, MANUAL, CLOSING
    }

    private State state;

    public FrontSliderSubsystem(HardwareMap hm, BooleanSupplier rightSup,
                                BooleanSupplier leftSup) {
        rightServo = hm.get(CRServoImplEx.class, "frontSlR");
        leftServo = hm.get(CRServoImplEx.class, "frontSlL");

        this.rightSup = rightSup;
        this.leftSup = leftSup;
        this.state = State.CLOSING;
    }

    public void periodic() {
        if ((rightSup.getAsBoolean() || leftSup.getAsBoolean()) && rightServo.getPower() < 0 ) {
            stop();
        } else if (state == State.CLOSING) {
            close();
        } else if (state == State.OPENING) {

        }
    }

    private void set(double power) {
        if ((rightSup.getAsBoolean() || leftSup.getAsBoolean()) && power < 0) {
            return;
        }
        rightServo.setPower(power);
        leftServo.setPower(-power);
//        if(!leftSup.getAsBoolean()) leftServo.setPower(-power);
//        else leftServo.setPower(0);
    }

    public void open(double power) {
        state = State.OPENING;
        set(power);
    }

    public void manual(double power) {
        state = State.MANUAL;
        set(power);
    }

    public void close() {
        state = State.CLOSING;
        set(-0.5);
    }

    public void stop() {
        rightServo.setPower(0);
        leftServo.setPower(0);
    }

    public void stopRight() {
        rightServo.setPower(0);
    }

    public void stopLeft() {
        leftServo.setPower(0);
    }

    public void setMaxPower(double pwr) {
        MAX_POWER = pwr;
    }

    public State getState() {
        return state;
    }

    public BooleanSupplier rightEnd() {
        return rightSup;
    }

    public BooleanSupplier leftEnd() {
        return leftSup;
    }
}
