package frc.robot.subsystems.drivetrain.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.drivers.ControllerDrivers;
import frc.robot.subsystems.drivetrain.drivers.PositionalDrivers;

public class FacePositionState extends AbstractDriveState<ControllerDrivers.YDriver, ControllerDrivers.XDriver, PositionalDrivers.ThetaDriver> {
    private static final PIDController THETA_CONTROLLER = new PIDController(4.0, 0, 0);

    private Translation2d position;

    public FacePositionState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Translation2d position) {
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
}
