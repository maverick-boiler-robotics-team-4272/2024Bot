package frc.robot.subsystems.shooter.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.Shooter;

public class LidarStoppedFeedState extends ShootState {
    public LidarStoppedFeedState(Shooter shooter, double percent) {
        super(shooter, -0.2, percent);
    }

    public LidarStoppedFeedState(Shooter shooter, DoubleSupplier percent) {
        this(shooter, percent.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.endLidarTripped();
    }
}
