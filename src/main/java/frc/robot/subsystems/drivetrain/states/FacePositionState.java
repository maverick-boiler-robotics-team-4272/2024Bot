package frc.robot.subsystems.drivetrain.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;

public class FacePositionState extends PositionalDriveState {
    private static final PIDController THETA_CONTROLLER = new PIDController(3.0, 0, 0);
    static {
        THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;
    private Translation2d position;

    public FacePositionState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Translation2d position) {
        super(drivetrain, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER);

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.position = position;
    }

    @Override
    public double getDesiredX() {
        return 0;
    }

    @Override
    public double getDesiredY() {
        return 0;
    }

    @Override
    public Rotation2d getDesiredTheta() {
        Translation2d robotPosition = requiredSubsystem.getRobotPose().getTranslation();

        return position.minus(robotPosition).getAngle();
        
    }

    @Override
    public double getXSpeed() {
        return ySpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
    }

    @Override
    public double getYSpeed() {
        return -xSpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
    }
}
