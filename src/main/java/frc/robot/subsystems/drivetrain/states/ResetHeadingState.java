package frc.robot.subsystems.drivetrain.states;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ResetHeadingState extends SetHeadingState {
    public ResetHeadingState(Drivetrain drivetrain) {
        super(drivetrain, new Rotation2d(0));
    }
}
