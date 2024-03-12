package frc.robot.subsystems.shooter.states;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class RevAndShootState extends State<Shooter> {
    private double revPower;
    private double feedPower;
    private BooleanSupplier startFeed;
    private boolean startedFeed;

    public RevAndShootState(Shooter shooter, double revPower, double feedPower, BooleanSupplier startFeed) {
        super(shooter);

        this.revPower = revPower;
        this.feedPower = feedPower;
        this.startFeed = startFeed;
        this.startedFeed = false;
    }

    @Override
    public void initialize() {
        requiredSubsystem.rev(revPower);
    }

    @Override
    public void execute() {
        if(startFeed.getAsBoolean() && !startedFeed) {
            requiredSubsystem.feed(feedPower);
            startedFeed = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.rev(0);
        requiredSubsystem.feed(0);
    }

    @Override
    public boolean isFinished() {
        return !requiredSubsystem.lidarTripped();
    }
}