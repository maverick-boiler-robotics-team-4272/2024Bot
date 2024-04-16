package frc.robot.subsystems.drivetrain.states;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.team4272.globals.State;

public class ResetHeadingState extends State<Drivetrain> {
    public ResetHeadingState(Drivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    public void initialize() {
        requiredSubsystem.zeroGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
