package frc.robot.subsystems.shooter.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class FeedState extends State<Shooter> {
    private DoubleSupplier percent;

    public FeedState(Shooter shooter, DoubleSupplier percent) {
        super(shooter);
        this.percent = percent;
    }

    @Override
    public void execute() {
        requiredSubsystem.feed(percent.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.feed(0.0);
    }
}
