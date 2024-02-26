package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class RevState extends State<Shooter> {
    private double power;

    public RevState(Shooter shooter, double power) {
        super(shooter);

        this.power = power;
    }

    @Override
    public void initialize() {
        requiredSubsystem.rev(power);
    }

    @Override
    public void end(boolean interrupted) {
        // if(!interrupted)
            requiredSubsystem.rev(0);
    }
}
