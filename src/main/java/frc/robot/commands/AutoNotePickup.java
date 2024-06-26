package frc.robot.commands;

import static frc.robot.constants.TelemetryConstants.Limelights.BACK_LIMELIGHT;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.drivers.ControllerDrivers;
import frc.robot.subsystems.drivetrain.drivers.PositionalDrivers;
import frc.robot.subsystems.drivetrain.states.AbstractDriveState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class AutoNotePickup extends AbstractDriveState<ControllerDrivers.YDriver, ControllerDrivers.XDriver, PositionalDrivers.ThetaDriver> {
    IntakeSubsystem intake;
    Shooter shooter;

    public AutoNotePickup(Drivetrain drivetrain, IntakeSubsystem intake, Shooter shooter, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        super(
            drivetrain,
            new ControllerDrivers.YDriver(ySpeed),
            new ControllerDrivers.XDriver(xSpeed),
            new PositionalDrivers.ThetaDriver(drivetrain, new PIDController(3.5, 0.0001, 0))
        );

        this.intake = intake;
        this.shooter = shooter;
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
            if(BACK_LIMELIGHT.getTY() < 0 && !shooter.lidarTripped()) {
                intake.runMotor(0.9);
                shooter.feed(0.9);
            } else if(shooter.beginLidarTripped()) {
                shooter.feed(0.7);
            } else {
                intake.runMotor(0);
                shooter.feed(0.0);
            }
        } else {
            thetaDriver.setDesiredAngle(requiredSubsystem.getRobotPose().getRotation());
            intake.runMotor(0);
            shooter.feed(0.0);
        }

        drive();
    }
}
