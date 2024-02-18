package frc.robot.subsystems.shooter.states;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.Shooter;

public class ShootState extends FeedState {
    DoubleSupplier rev;
    
    public ShootState(Shooter shooter, DoubleSupplier rev, BooleanSupplier feed) {
        super(shooter, () -> feed.getAsBoolean() ? 0.5 : 0);

        this.rev = rev;
    }

    @Override
    public void execute() {
        super.execute();
        requiredSubsystem.rev(rev.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        requiredSubsystem.rev(0.0);
    }
}
