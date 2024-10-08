package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.TelemetryConstants.Limelights.BACK_LIMELIGHT;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.drivers.ControllerDrivers;
import frc.robot.subsystems.drivetrain.drivers.PositionalDrivers;

public class NoteLockState extends AbstractDriveState<ControllerDrivers.YDriver, ControllerDrivers.XDriver, PositionalDrivers.ThetaDriver> {
    public NoteLockState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        super(
            drivetrain,
            new ControllerDrivers.YDriver(ySpeed),
            new ControllerDrivers.XDriver(xSpeed),
            new PositionalDrivers.ThetaDriver(drivetrain, new PIDController(3.5, 0.0001, 0))
        );
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public boolean isFieldRelative() {
        return true;
    }

    @Override
    public void execute() {
        
        if(BACK_LIMELIGHT.getTV()) {
            thetaDriver.setDesiredAngle(Rotation2d.fromDegrees(BACK_LIMELIGHT.getTX()).minus(requiredSubsystem.getRobotPose().getRotation()).unaryMinus());
        } else {
            thetaDriver.setDesiredAngle(requiredSubsystem.getRobotPose().getRotation());
        }

        drive();
    }
}
