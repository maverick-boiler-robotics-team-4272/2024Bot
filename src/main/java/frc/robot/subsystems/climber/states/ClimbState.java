package frc.robot.subsystems.climber.states;

import static frc.robot.constants.RobotConstants.ClimberConstants.CLIMBER_MAX_HEIGHT;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.climber.Climber;
import frc.team4272.globals.State;

public class ClimbState extends State<Climber> {
    private DoubleSupplier power;

    public ClimbState(Climber climber, DoubleSupplier power) {
        super(climber);

        this.power = power;
    }

    @Override
    public void execute() {
        requiredSubsystem.setClimerHeight(-Math.round(power.getAsDouble()) * CLIMBER_MAX_HEIGHT);
    }
}
