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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Loggable;
import frc.robot.utils.Pigeon;
import frc.robot.utils.SwerveModule;
import frc.team4272.swerve.utils.SwerveDriveBase;
import frc.team4272.swerve.utils.SwerveModuleBase.PositionedSwerveModule;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.SwerveModuleConstants.MAX_MODULE_SPEED;
import static frc.robot.constants.TelemetryConstants.Limelights.CENTER_LIMELIGHT;
import static frc.robot.constants.UniversalConstants.*;


public class Drivetrain extends SwerveDriveBase<Pigeon, SwerveModule> implements Loggable {
    @AutoLog
    public static class DrivetrainInputs {
        public Pose2d odometryPose;
        public Pose2d limelightPose;
        public Pose2d estimatedPose;
        public Pose2d desiredPose;

        public Pose2d filteredPose;
    }

    private DrivetrainInputsAutoLogged drivetrainInputs;

    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;

    private Pose2d lastPos;


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
        drivetrainInputs.desiredPose = new Pose2d();

        drivetrainInputs.filteredPose = new Pose2d();
        lastPos = new Pose2d();

        odometry = new SwerveDriveOdometry(kinematics, gyroscope.getRotation(), getPositions());
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyroscope.getRotation(),
            getPositions(),
            CENTER_LIMELIGHT.getRobotPose(),
            VecBuilder.fill(0.5, 0.5, 0.5), // Guestimations to try and make tracking better
            VecBuilder.fill(0.75, 0.75, 0.65) // Computed standard deviations, (~worst case / 2)
        );

        setMaxSpeeds(MAX_TRANSLATIONAL_SPEED, MAX_ROTATIONAL_SPEED, MAX_MODULE_SPEED);
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        super.drive(speeds);
        
        updateOdometry();
    }

    public void updateOdometry() {
        drivetrainInputs.odometryPose = odometry.update(gyroscope.getRotation().unaryMinus(), getPositions());

        poseEstimator.update(gyroscope.getRotation().unaryMinus(), getPositions());
        if(
            CENTER_LIMELIGHT.isValidTarget()
        ) {
            poseEstimator.addVisionMeasurement(drivetrainInputs.limelightPose, Timer.getFPGATimestamp());
        }

        drivetrainInputs.estimatedPose = poseEstimator.getEstimatedPosition();
    }

    public Pose2d getRobotPose() { 
        return drivetrainInputs.estimatedPose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getOdometryPose() {
        return drivetrainInputs.odometryPose;
    }

    public void setLoggedDesiredPose(Pose2d pose) {
        drivetrainInputs.desiredPose = pose;
    }

    public void setCoastMode(boolean coast) {
        for(SwerveModule m : modules) {
            m.setCoastMode(coast);
        }
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
        CENTER_LIMELIGHT.filterPosition();
        log("Subsystems", "Drivetrain");

        drivetrainInputs.limelightPose =  CENTER_LIMELIGHT.getUnfilteredPose();
        

        if(
            !drivetrainInputs.limelightPose.equals(new Pose2d(FIELD_HALF_WIDTH_METERS, FIELD_HALF_HEIGHT_METERS, new Rotation2d(0))) &&
           !lastPos.equals(drivetrainInputs.limelightPose)
        ) {
            drivetrainInputs.filteredPose = CENTER_LIMELIGHT.getRobotPose();
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for(int i = 0; i < numModules; i++) {
            states[i] = modules[i].getState();
        }

        return states;
        
    }
}
