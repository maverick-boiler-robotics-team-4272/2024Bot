package frc.robot.subsystems.shooter.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.Shooter;

public class LidarStoppedFeedState extends FeedState {
    public LidarStoppedFeedState(Shooter shooter, DoubleSupplier percent) {
        super(shooter, percent);
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.lidarTripped();
    }
}
