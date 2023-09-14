package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;


public class FreightFrenzyRobot extends RobotEx {

    private BucketSubsystem bucket;
//    private CarouselSubsystem carousel;
//    private IntakeSubsystem intake;
//    private SliderSubsystem slider;

    public FreightFrenzyRobot(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                     GamepadEx toolOp) {
        super(hardwareMap, telemetry, driverOp, toolOp);
    }

    @Override
    public void initMechanisms(HardwareMap hardwareMap) {
        ////////////////////////////////////////// Bucket //////////////////////////////////////////
        bucket = new BucketSubsystem(hardwareMap);
        toolOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new BucketCommand(bucket));

        ///////////////////////////////////////// Carousel /////////////////////////////////////////
//        carousel = new CarouselSubsystem(hardwareMap);
//
//        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5)
//                .whileActiveContinuous(new InstantCommand(carousel::clockwiseTurn, carousel))
//                .whenInactive(new InstantCommand(carousel::stop, carousel));
//        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5)
//                .whileActiveContinuous(new InstantCommand(carousel::counterClockwiseTurn, carousel))
//                .whenInactive(new InstantCommand(carousel::stop, carousel));
//        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .toggleWhenPressed(
//                        new PerpetualCommand(new SequentialCommandGroup(
//                                new InstantCommand(carousel::clockwiseTurn, carousel),
//                                new WaitCommand(5000),
//                                new InstantCommand(carousel::stop, carousel),
//                                new WaitCommand(1000))),
//                        new InstantCommand(carousel::stop, carousel)
//                );
        // TODO: check what happens if CarouselCommand is running and RIGHT/LEFT_TRIGGER is pressed

        ////////////////////////////////////////// Slider //////////////////////////////////////////
//        slider = new SliderSubsystem(hardwareMap);
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.THREE));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.TWO));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.PARK));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.ONE));

        ////////////////////////////////////////// Intake //////////////////////////////////////////
//        intake = new IntakeSubsystem(hardwareMap);
//        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .toggleWhenPressed(
//                        new SequentialCommandGroup(
//                                new InstantCommand(bucket::intake, bucket),
//                                new SliderCommand(slider, SliderSubsystem.Level.INTAKE),
//                                new PerpetualCommand(new InstantCommand(intake::run, intake))),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(intake::reverse, intake),
//                                        new WaitCommand(500),
//                                        new InstantCommand(intake::stop, intake)),
//                                new InstantCommand(bucket::rest, bucket),
//                                new SliderCommand(slider, SliderSubsystem.Level.PARK)
//                        )
//                );
    }
}