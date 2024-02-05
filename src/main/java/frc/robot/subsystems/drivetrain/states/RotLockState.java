package frc.robot.subsystems.drivetrain.states;

import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.MAX_TRANSLATIONAL_SPEED;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotLockState extends PositionalDriveState {
    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;
    Supplier<Rotation2d> theta;

    public RotLockState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Supplier<Rotation2d> theta) {
        super(drivetrain, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER);

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.theta = theta;
    }

    @Override
    public double getDesiredX() {
        return 0;
    }

    @Override
    public double getDesiredY() {
        return 0;
    }

    @Override
    public Rotation2d getDesiredTheta() {
        return theta.get();
    }

    @Override
    public double getXSpeed() {
        return ySpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
    }

    @Override
    public double getYSpeed() {
        return -xSpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
    }
}
