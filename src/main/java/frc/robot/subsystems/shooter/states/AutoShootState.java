package frc.robot.subsystems.shooter.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootState extends SequentialCommandGroup {
    public AutoShootState(Shooter shooter, double shootPower, double feedPower) {
        super(
            new LidarStoppedFeedState(shooter, 0.2),
            new RevState(shooter, shootPower).withTimeout(0.5),
            new ShootState(shooter, shootPower, feedPower, true)
        );
    }
}
