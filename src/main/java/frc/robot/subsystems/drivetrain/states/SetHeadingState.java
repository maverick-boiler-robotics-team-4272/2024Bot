package frc.robot.subsystems.drivetrain.states;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.team4272.globals.State;

public class SetHeadingState extends State<Drivetrain> {
    private Rotation2d heading;
    
    public SetHeadingState(Drivetrain drivetrain, Rotation2d heading) {
        super(drivetrain);

        this.heading = heading;
    }

    @Override
    public void initialize() {
        requiredSubsystem.setGyroscopeReading(heading);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
