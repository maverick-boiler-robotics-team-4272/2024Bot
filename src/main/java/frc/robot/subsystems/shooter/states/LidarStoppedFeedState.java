package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;

import static frc.robot.utils.misc.BEAN.*;

public class LidarStoppedFeedState extends ShootState {
    public LidarStoppedFeedState(Shooter shooter, double revSpeed, double feedSpeed) {
        super(shooter, revSpeed, feedSpeed);
    }

    public LidarStoppedFeedState(Shooter shooter, double percent) {
        this(shooter, LIMA_BEAN.stalk().beans, percent);
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.endLidarTripped();
    }
}
