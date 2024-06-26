package frc.robot.constants;


import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.logging.Norm;
import frc.robot.utils.logging.Norm.Metric;

import static frc.robot.constants.TelemetryConstants.Limelights.FRONT_LIMELIGHT;

public class Norms {
    private Norms() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    private static Metric<Pose2d> poseNorm = (a, b) -> {
        return Math.hypot(a.getTranslation().getDistance(b.getTranslation()), a.getRotation().getRadians() - b.getRotation().getRadians());
    };

    private static Norm<Pose2d> autoNorm = null;
    private static Norm<Pose2d> limelightNorm = null;
    private static Norm<Double> armPidNorm = null;

    public static Norm<Pose2d> getAutoNorm() {
        if(autoNorm == null)
            throw new IllegalStateException("Initialize has not been called.");
        return autoNorm;
    }

    public static Norm<Pose2d> getLimelightNorm() {
        if(limelightNorm == null)
            throw new IllegalStateException("Initialize has not been called.");
        return limelightNorm;
    }

    public static Norm<Double> getArmNorm() {
        if(armPidNorm == null)
            throw new IllegalStateException("Initialize has not been called");
        return armPidNorm;
    }

    public static void initialize(Drivetrain drivetrain, ArmElevatorSubsystem armElevator) {
        autoNorm = new Norm<>(drivetrain::getRobotPose, drivetrain::getDesiredPose, poseNorm, "AutoNorms");

        limelightNorm = new Norm<>(FRONT_LIMELIGHT::getRobotPose, FRONT_LIMELIGHT::getUnfilteredPose, poseNorm, "Limelight Norm");
        limelightNorm.reset();
        limelightNorm.enable();

        armPidNorm = new Norm<>(armElevator::getShooterRotation, armElevator::getShooterDesiredRotation, (a, b) -> a - b, "Arm PID Norm");
    }
}
