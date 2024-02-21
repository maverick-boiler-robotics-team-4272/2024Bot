package frc.robot.subsystems.shooter.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.Shooter;
import frc.team4272.globals.State;

public class OutfeedState extends State<Shooter> {
    private DoubleSupplier power;
    
    public OutfeedState(Shooter shooter, DoubleSupplier power) {
        super(shooter);

        this.power = power;
    }

    @Override
    public void execute() {
        requiredSubsystem.feed(-power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.feed(0);
    }
}
