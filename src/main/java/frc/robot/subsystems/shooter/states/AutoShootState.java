package frc.robot.subsystems.shooter.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootState extends SequentialCommandGroup {
    public AutoShootState(Shooter shooter) {
        super(
            new ShootState(shooter, () -> 1.0, () -> false).withTimeout(1),
            new ShootState(shooter, () -> 1.0, () -> true)
        );
    }
}
