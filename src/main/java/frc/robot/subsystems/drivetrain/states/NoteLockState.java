package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.RobotConstants.DrivetrainConstants.MAX_TRANSLATIONAL_SPEED;
import static frc.robot.constants.TelemetryConstants.Limelights.BACK_LIMELIGHT;
import static frc.robot.constants.UniversalConstants.getGlobalPositions;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.team4272.globals.MathUtils;

public class NoteLockState extends AbstractDriveState {
    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;
    private Rotation2d sourceRotation;

    public NoteLockState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        super(drivetrain);

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    @Override
    public void initialize() {
        super.initialize();
        sourceRotation = getGlobalPositions().TO_SOURCE;
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
        if(BACK_LIMELIGHT.getTV()) {
            return BACK_LIMELIGHT.getTX() * 0.1;
        } else {
            return MathUtils.inputModulo(requiredSubsystem.getRobotPose().getRotation().minus(sourceRotation).getRadians(), -Math.PI, Math.PI) * 2.5;
        }
    }

    @Override
    public boolean isFieldRelative() {
        return true;
    }
}
