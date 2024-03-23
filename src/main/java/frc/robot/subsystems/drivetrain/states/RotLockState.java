package frc.robot.subsystems.drivetrain.states;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.drivers.ControllerDrivers;
import frc.robot.subsystems.drivetrain.drivers.PositionalDrivers;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.MAX_TRANSLATIONAL_SPEED;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotLockState extends AbstractDriveState<ControllerDrivers.YDriver, ControllerDrivers.XDriver, PositionalDrivers.ThetaDriver> {
    Supplier<Rotation2d> theta;

    public RotLockState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Supplier<Rotation2d> theta) {
        super(
            drivetrain,
            new ControllerDrivers.YDriver(ySpeed),
            new ControllerDrivers.XDriver(xSpeed),
            new PositionalDrivers.ThetaDriver(drivetrain)
        );

        this.theta = theta;
    }

    @Override
    public void execute() {
        thetaDriver.setDesiredAngle(theta.get());

        drive();
    }
}
