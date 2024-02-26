package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class ShootState extends State<Shooter> {
    private double shootPower;
    private double feedPower;
    
    public ShootState(Shooter shooter, double shootPower, double feedPower) {
        super(shooter);

        this.shootPower = shootPower;
        this.feedPower = feedPower;
    }

    @Override
    public void initialize() {
        requiredSubsystem.rev(shootPower);
        requiredSubsystem.feed(feedPower);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.feed(0.0);
        requiredSubsystem.rev(0.0);
    }
}
