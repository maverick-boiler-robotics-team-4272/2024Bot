package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;

public class LidarStoppedFeedState extends ShootState {
    public LidarStoppedFeedState(Shooter shooter, double percent) {
        super(shooter, -0.1, percent);
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.endLidarTripped();
    }
}
