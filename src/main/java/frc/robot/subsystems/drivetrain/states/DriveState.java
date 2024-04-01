package frc.robot.subsystems.drivetrain.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.drivers.ControllerDrivers;

public class DriveState extends AbstractDriveState<ControllerDrivers.YDriver, ControllerDrivers.XDriver, ControllerDrivers.ThetaDriver> {
    public DriveState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier thetaSpeed) {
        super(
            drivetrain,
            new ControllerDrivers.YDriver(ySpeed),
            new ControllerDrivers.XDriver(xSpeed),
            new ControllerDrivers.ThetaDriver(thetaSpeed)
        );
    }

    @Override
    public boolean isFieldRelative() {
        return true;
    }
}
