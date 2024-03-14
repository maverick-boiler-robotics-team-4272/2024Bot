package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class FeedState extends State<Shooter> {
    private double percent;

    public FeedState(Shooter shooter, double percent) {
        super(shooter);
        this.percent = percent;
    }

    @Override
    public void initialize() {
        requiredSubsystem.feed(percent);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.feed(0.0);
    }
}
