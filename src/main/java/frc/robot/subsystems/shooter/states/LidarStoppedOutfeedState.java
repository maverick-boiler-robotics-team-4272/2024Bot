package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;

public class LidarStoppedOutfeedState extends OutfeedState {
    public LidarStoppedOutfeedState(Shooter shooter, double power) {
        super(shooter, power);
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.beginLidarTripped();
    }
}
