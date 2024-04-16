package frc.robot.subsystems.drivetrain.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.drivers.ControllerDrivers;
import frc.robot.subsystems.drivetrain.drivers.PositionalDrivers;

public class AngleStoppedFacePositionState extends AbstractDriveState<ControllerDrivers.YDriver, ControllerDrivers.XDriver, PositionalDrivers.ThetaDriver> {
    private static final PIDController THETA_CONTROLLER = new PIDController(3.5, 0, 0);
    private static final Rotation2d ANGLE_DELTA = Rotation2d.fromDegrees(5.0);

    private Translation2d position;

    public AngleStoppedFacePositionState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Translation2d position) {
        super(
            drivetrain,
            new ControllerDrivers.YDriver(ySpeed),
            new ControllerDrivers.XDriver(xSpeed),
            new PositionalDrivers.ThetaDriver(drivetrain, THETA_CONTROLLER)
        );

        this.position = position;
    }

    @Override
    public boolean isFieldRelative() {
        return true;
    }

    @Override
    public void execute() {
        thetaDriver.setDesiredAngle(position.minus(requiredSubsystem.getRobotPose().getTranslation()).getAngle());

        drive();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(thetaDriver.getDesiredAngle().minus(requiredSubsystem.getRobotPose().getRotation()).getRadians()) < ANGLE_DELTA.getRadians();
    }
}
