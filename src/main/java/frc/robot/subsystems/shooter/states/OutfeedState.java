package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class OutfeedState extends State<Shooter> {
    private double power;
    
    public OutfeedState(Shooter shooter, double power) {
        super(shooter);

        this.power = power;
    }

    @Override
    public void initialize() {
        requiredSubsystem.feed(-power);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.feed(0);
    }
}
