package frc.robot.subsystems.drivetrain.states;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

import frc.robot.subsystems.drivetrain.drivers.PositionalDrivers;


public class PositionState extends AbstractDriveState<PositionalDrivers.XDriver, PositionalDrivers.YDriver, PositionalDrivers.ThetaDriver> {
    Supplier<Pose2d> pose;

    public PositionState(Drivetrain drivetrain, Supplier<Pose2d> pose) {
        super(
            drivetrain,
            new PositionalDrivers.XDriver(drivetrain),
            new PositionalDrivers.YDriver(drivetrain),
            new PositionalDrivers.ThetaDriver(drivetrain)
        );

        this.pose = pose;
    }

    @Override
    public boolean isFieldRelative() {
        return true;
    }

    @Override
    public void execute() {
        Pose2d desiredPose = pose.get();

        xDriver.setDesiredXPosition(desiredPose.getX());
        yDriver.setDesiredYPosition(desiredPose.getY());
        thetaDriver.setDesiredAngle(desiredPose.getRotation());

        drive();
    }
}
