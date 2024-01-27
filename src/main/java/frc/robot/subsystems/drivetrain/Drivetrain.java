package frc.robot.subsystems.drivetrain;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Loggable;
import frc.robot.utils.Pigeon;
import frc.robot.utils.SwerveModule;
import frc.team4272.swerve.utils.SwerveDriveBase;
import frc.team4272.swerve.utils.SwerveModuleBase.PositionedSwerveModule;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;
import static frc.robot.constants.TelemetryConstants.Limelights.CENTER_LIMELIGHT;
import static frc.robot.constants.UniversalConstants.*;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;


public class Drivetrain extends SwerveDriveBase<Pigeon, SwerveModule> implements Loggable {
    @AutoLog
    public static class DrivetrainInputs {
        public Pose2d odometryPose;
        public Pose2d limelightPose;
        public Pose2d estimatedPose;
    }

    private DrivetrainInputsAutoLogged drivetrainInputs;

    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;

    public Drivetrain() {
        super(
            new Pigeon(PIGEON_ID),
            SwerveModule.class,
            List.of(
                new PositionedSwerveModule<SwerveModule>(new SwerveModule(FRONT_LEFT_MODULE_ID,  FRONT_LEFT_OFFSET),  FRONT_LEFT_POSITION),
                new PositionedSwerveModule<SwerveModule>(new SwerveModule(FRONT_RIGHT_MODULE_ID, FRONT_RIGHT_OFFSET), FRONT_RIGHT_POSITION),
                new PositionedSwerveModule<SwerveModule>(new SwerveModule(BACK_LEFT_MODULE_ID,   BACK_LEFT_OFFSET),   BACK_LEFT_POSITION),
                new PositionedSwerveModule<SwerveModule>(new SwerveModule(BACK_RIGHT_MODULE_ID,  BACK_RIGHT_OFFSET),  BACK_RIGHT_POSITION)
            )
        );

        drivetrainInputs = new DrivetrainInputsAutoLogged();

        drivetrainInputs.odometryPose = new Pose2d();
        drivetrainInputs.estimatedPose = new Pose2d();
        drivetrainInputs.limelightPose = new Pose2d();

        odometry = new SwerveDriveOdometry(kinematics, gyroscope.getRotation(), getPositions());
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyroscope.getRotation(),
            getPositions(),
            CENTER_LIMELIGHT.getRobotPose(),
            VecBuilder.fill(0.5, 0.5, 0.5), // Guestimations to try and make tracking better
            VecBuilder.fill(1.0, 1.0, 0.65) // Computed standard deviations, (~worst case / 2)
        );

        setMaxSpeeds(Units.MetersPerSecond.convertFrom(14.5, Units.FeetPerSecond));
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        super.drive(speeds);
        
        updateOdometry();
    }

    public void updateOdometry() {
        drivetrainInputs.odometryPose = odometry.update(gyroscope.getRotation().unaryMinus(), getPositions());

        poseEstimator.update(gyroscope.getRotation().unaryMinus(), getPositions());
        if(CENTER_LIMELIGHT.isValidTarget() && !drivetrainInputs.limelightPose.equals(new Pose2d(FIELD_HALF_WIDTH_METERS, FIELD_HALF_HEIGHT_METERS, new Rotation2d(0))))
            poseEstimator.addVisionMeasurement(drivetrainInputs.limelightPose, Timer.getFPGATimestamp());

        drivetrainInputs.estimatedPose = poseEstimator.getEstimatedPosition();
    }

    public Pose2d getRobotPose() {
        return drivetrainInputs.estimatedPose;
    }

    public Pose2d getOdometryPose() {
        return drivetrainInputs.odometryPose;
    }

    public void setRobotPose(Pose2d pose) {
        odometry.resetPosition(gyroscope.getRotation(), getPositions(), pose);
        poseEstimator.resetPosition(gyroscope.getRotation(), getPositions(), pose);
    }

    public void setToZero() {
        setRobotPose(new Pose2d(FIELD_HALF_WIDTH_METERS, FIELD_HALF_HEIGHT_METERS, new Rotation2d(0.0)));
    }

    public void setGyroscopeReading(Rotation2d heading) {
        gyroscope.setRotation(heading);

        odometry.resetPosition(gyroscope.getRotation(), getPositions(), odometry.getPoseMeters());
        poseEstimator.resetPosition(gyroscope.getRotation(), getPositions(), poseEstimator.getEstimatedPosition());
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].log(subdirectory + "/" + humanReadableName, "Module" + i);
        }

        gyroscope.log(subdirectory + "/" + humanReadableName, "Pigeon");

        Logger.processInputs(subdirectory + "/" + humanReadableName, drivetrainInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Drivetrain");

        drivetrainInputs.limelightPose =  CENTER_LIMELIGHT.getRobotPose();
    }
}
