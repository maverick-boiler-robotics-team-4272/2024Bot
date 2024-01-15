package frc.robot.subsystems.drivetrain;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Loggable;
import frc.robot.utils.Pigeon;
import frc.robot.utils.SwerveModule;
import frc.team4272.swerve.utils.SwerveDriveBase;
import frc.team4272.swerve.utils.SwerveModuleBase.PositionedSwerveModule;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;
import static frc.robot.constants.TelemetryConstants.Limelights.CENTER_LIMELIGHT;

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

        odometry = new SwerveDriveOdometry(kinematics, gyroscope.getRotation(), getPositions());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroscope.getRotation(), getPositions(), CENTER_LIMELIGHT.getRobotPose());

        setMaxSpeeds(Units.MetersPerSecond.convertFrom(14.5, Units.FeetPerSecond));
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        super.drive(speeds);
        
        updateOdometry();
    }

    public void updateOdometry() {
        drivetrainInputs.odometryPose = odometry.update(gyroscope.getRotation().unaryMinus(), getPositions());
        drivetrainInputs.limelightPose =  CENTER_LIMELIGHT.getRobotPose();

        poseEstimator.update(gyroscope.getRotation().unaryMinus(), getPositions());
        poseEstimator.addVisionMeasurement(drivetrainInputs.limelightPose, Timer.getFPGATimestamp());

        drivetrainInputs.estimatedPose = poseEstimator.getEstimatedPosition();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        if(DriverStation.isDisabled())
            return; // Don't want any logging while disabled, because that will just increase file size
        for(int i = 0; i < modules.length; i++) {
            modules[i].log(subdirectory + "/" + humanReadableName, "Module" + i);
        }

        gyroscope.log(subdirectory + "/" + humanReadableName, "Pigeon");

        Logger.processInputs(subdirectory + "/" + humanReadableName, drivetrainInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Drivetrain");
    }
}
