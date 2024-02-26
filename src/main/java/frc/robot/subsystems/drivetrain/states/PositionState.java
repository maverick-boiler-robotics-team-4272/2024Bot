package frc.robot.subsystems.drivetrain.states;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;

public class PositionState extends PositionalDriveState {
    Supplier<Pose2d> pose;

    public PositionState(Drivetrain drivetrain, Supplier<Pose2d> pose) {
        super(drivetrain, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER);

        this.pose = pose;
    }

    @Override
    public double getDesiredX() {
        return pose.get().getX();
    }

    @Override
    public double getDesiredY() {
        return pose.get().getY();
    }

    @Override
    public Rotation2d getDesiredTheta() {
        return pose.get().getRotation();
    }
}
