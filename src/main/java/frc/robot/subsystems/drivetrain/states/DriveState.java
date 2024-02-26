package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;

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
        return ySpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
    }

    @Override
    public double getYSpeed() {
        return -xSpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
    }

    @Override
    public double getThetaSpeed() {
        return thetaSpeed.getAsDouble() * MAX_ROTATIONAL_SPEED;
    }
}
