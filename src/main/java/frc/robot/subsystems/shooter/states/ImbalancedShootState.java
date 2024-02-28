package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class ImbalancedShootState extends State<Shooter> {
    private double shootPower1;
    private double shootPower2;
    private double feedPower;
    
    public ImbalancedShootState(Shooter shooter, double shootPower1, double shootPower2, double feedPower) {
        super(shooter);

        this.shootPower1 = shootPower1;
        this.shootPower2 = shootPower2;
        this.feedPower = feedPower;
    }

    @Override
    public void initialize() {
        requiredSubsystem.runMotors(shootPower1, shootPower2);
        requiredSubsystem.feed(feedPower);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.feed(0.0);
        requiredSubsystem.rev(0.0);
    }
}
