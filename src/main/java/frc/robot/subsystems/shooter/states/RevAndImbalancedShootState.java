package frc.robot.subsystems.shooter.states;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class RevAndImbalancedShootState extends State<Shooter> {
    private double revPower1;
    private double revPower2;
    private double feedPower;
    private boolean useLidar;

    private BooleanSupplier startFeed;
    private boolean startedFeed;

    public RevAndImbalancedShootState(Shooter shooter, double revPower1, double revPower2, double feedPower, boolean useLidar, BooleanSupplier startFeed) {
        super(shooter);

        this.revPower1 = revPower1;
        this.revPower2 = revPower2;
        this.feedPower = feedPower;
        this.useLidar = useLidar;
        this.startFeed = startFeed;
        this.startedFeed = false;
    }

    public RevAndImbalancedShootState(Shooter shooter, double revPower1, double revPower2, double feedPower, BooleanSupplier startFeed) {
        this(shooter, revPower1, revPower2, feedPower, false, startFeed);
    }

    @Override
    public void initialize() {
        requiredSubsystem.runMotors(revPower1, revPower2);
        startedFeed = false;
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
        return useLidar && !requiredSubsystem.lidarTripped();
    }
}
