package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.CommandBase;


public class BucketCommand extends CommandBase {
    private final BucketSubsystem bucket;

    public BucketCommand(BucketSubsystem bucket) {
        this.bucket = bucket;
        addRequirements(bucket);
    }

    @Override
    public void initialize() {
        BucketSubsystem.State state = bucket.getState();

        if (state == BucketSubsystem.State.REST)
            bucket.release();
        else if (state == BucketSubsystem.State.RELEASE)
            bucket.rest();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
