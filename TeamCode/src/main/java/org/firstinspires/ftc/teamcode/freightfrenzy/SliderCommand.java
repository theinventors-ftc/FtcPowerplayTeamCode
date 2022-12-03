package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.CommandBase;


public class SliderCommand extends CommandBase {
    private final SliderSubsystem slider;
    private final SliderSubsystem.Level level;

    public SliderCommand(SliderSubsystem slider, SliderSubsystem.Level level) {
        this.slider = slider;
        this.level = level;
        addRequirements(this.slider);
    }

    @Override
    public void initialize() {
        slider.setLevel(level);
    }

    @Override
    public void execute() {
        slider.run();
    }

    @Override
    public void end(boolean interrupted) {
        slider.stop();
    }

    @Override
    public boolean isFinished() {
        return slider.atTargetLevel();
    }
}
