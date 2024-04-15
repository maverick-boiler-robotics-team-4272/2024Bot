package frc.robot.subsystems.shooter.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

import static frc.robot.utils.misc.BEAN.*;

public class AutoShootState extends SequentialCommandGroup {
    public AutoShootState(Shooter shooter, double shootPower, double feedPower, double delay) {
        super(
            new LidarStoppedFeedState(shooter, LIMA_BEAN.dip().beans).withTimeout(1.0),
            new RevState(shooter, shootPower).withTimeout(delay),
            new ShootState(shooter, shootPower, feedPower, true).withTimeout(2.0)
        );
    }

    public AutoShootState(Shooter shooter, double shootPower, double feedPower) {
        this(shooter, shootPower, feedPower, 0.5);
    }
}
