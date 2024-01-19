package frc.robot.subsystems.drivetrain.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveState extends AbstractDriveState {
    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;
    private DoubleSupplier thetaSpeed;

    public DriveState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier thetaSpeed) {
        super(drivetrain);

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.thetaSpeed = thetaSpeed;
    }

    @Override
    public boolean isFieldRelative() {
        return true;
    }

    @Override
    public double getXSpeed() {
        return ySpeed.getAsDouble();
    }

    @Override
    public double getYSpeed() {
        return -xSpeed.getAsDouble();
    }

    @Override
    public double getThetaSpeed() {
        return thetaSpeed.getAsDouble();
    }
}
