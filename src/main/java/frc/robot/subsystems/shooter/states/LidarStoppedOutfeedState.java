package frc.robot.subsystems.shooter.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.Shooter;

public class LidarStoppedOutfeedState extends OutfeedState {
    public LidarStoppedOutfeedState(Shooter shooter, DoubleSupplier power) {
        super(shooter, power);
    }

    @Override
    public boolean isFinished() {
        return !requiredSubsystem.lidarTripped();
    }
}
