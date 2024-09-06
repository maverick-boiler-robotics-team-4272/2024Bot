package frc.robot.subsystems.drivetrain;

import java.util.*;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;

// Math
import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
// Timing
import edu.wpi.first.wpilibj.Timer;
import frc.robot.limelight.LimelightHelpers;
// Hardware
import frc.robot.utils.hardware.*;
import frc.team4272.swerve.utils.*;
import frc.team4272.swerve.utils.SwerveModuleBase.PositionedSwerveModule;

import static frc.robot.constants.AutoConstants.Paths.CONTAINER_CHOOSER;
// Constants
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;
import static frc.robot.constants.RobotConstants.LimelightConstants.*;
import static frc.robot.constants.RobotConstants.DrivetrainConstants.SwerveModuleConstants.*;
import static frc.robot.constants.TelemetryConstants.Limelights.*;
import static frc.robot.constants.UniversalConstants.*;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;

public class Drivetrain extends SwerveDriveBase<Pigeon, SwerveModule> implements Loggable {
    @AutoLog
    public static class DrivetrainInputs {
        public Pose2d estimatedPose;
        public Pose2d desiredPose;

        public double speakerDistance;

        public SwerveModuleState[] currentStates;
        public SwerveModuleState[] setStates;

        public Pose2d notePose;
    }

    private DrivetrainInputsAutoLogged drivetrainInputs;

    private SwerveDrivePoseEstimator poseEstimator;

    private boolean fuseVision = true;

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

        drivetrainInputs.estimatedPose = new Pose2d();
        drivetrainInputs.desiredPose = new Pose2d();
        drivetrainInputs.notePose = new Pose2d();

        drivetrainInputs.currentStates = new SwerveModuleState[4];
        drivetrainInputs.setStates = new SwerveModuleState[4];

        for(int i = 0; i < 4; i++) {
            drivetrainInputs.currentStates[i] = new SwerveModuleState();
            drivetrainInputs.setStates[i] = new SwerveModuleState();
        }

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyroscope.getRotation(),
            getPositions(),
            FRONT_LIMELIGHT.getRobotPose(),
            VecBuilder.fill(0.5, 0.5, 0.5), // Guestimations to try and make tracking better
            VecBuilder.fill(0.7, 0.7, 999999) // Computed standard deviations, (~worst case / 2)
        );

        setMaxSpeeds(MAX_TRANSLATIONAL_SPEED, MAX_ROTATIONAL_SPEED, MAX_MODULE_SPEED);
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        super.drive(speeds);
        
        updateOdometry();
    }

    @Override
    public void setStates(SwerveModuleState... states) {
        if(states.length != numModules) throw new IllegalArgumentException("Number of states provided doesnt match number of modules");

        for(int i = 0; i < states.length; i++) {
            modules[i].goToState(states[i]);
            drivetrainInputs.setStates[i] = states[i];
        }
    }

    public void updateOdometry() {
        LimelightHelpers.PoseEstimate limelightMeasurement = FRONT_LIMELIGHT.getPoseEstimate(CONTAINER_CHOOSER.getSelected() == "Red");

        // if(limelightMeasurement != null) {
        //     if(limelightMeasurement.tagCount >= 2) {
        //         // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        //         poseEstimator.addVisionMeasurement(
        //             limelightMeasurement.pose,
        //             limelightMeasurement.timestampSeconds
        //         );
        //     }
        // }

        drivetrainInputs.estimatedPose = poseEstimator.getEstimatedPosition();
    }

    public Pose2d getRobotPose() { 
        return drivetrainInputs.estimatedPose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setDesiredPose(Pose2d pose) {
        drivetrainInputs.desiredPose = pose;
    }

    public Pose2d getDesiredPose() {
        return drivetrainInputs.desiredPose;
    }

    public Pose2d getNotePose() {
        return drivetrainInputs.notePose;
    }

    public void setCoastMode(boolean coast) {
        for(SwerveModule m : modules) {
            m.setCoastMode(coast);
        }
    }

    public void setRobotPose(Pose2d pose) {
        poseEstimator.resetPosition(gyroscope.getRotation().unaryMinus(), getPositions(), pose);
    }

    public void setToZero() {
        setRobotPose(new Pose2d(FIELD_HALF_WIDTH_METERS, FIELD_HALF_HEIGHT_METERS, new Rotation2d(0.0)));
    }

    public void setGyroscopeReading(Rotation2d heading) {
        gyroscope.setRotation(heading);

        setRobotPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), heading));
    }

    public void zeroGyro() {
        gyroscope.setRotation(new Rotation2d(0.0));

        setRobotPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), getGlobalPositions().TO_DRIVERSTATION.plus(new Rotation2d(Math.PI))));
    }

    public void enableVisionFusion() {
        fuseVision = true;
    }

    public void disableVisionFusion() {
        fuseVision = false;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].log(subdirectory + "/" + humanReadableName, "Module" + i);
            drivetrainInputs.currentStates[i] = modules[i].getState();
        }

        if(hasGlobalPositions()) {
            drivetrainInputs.speakerDistance = getGlobalPositions().SPEAKER_POSITION.getDistance(getRobotPose().getTranslation());
        } else {
            drivetrainInputs.speakerDistance = 0.0;
        }

        if(BACK_LIMELIGHT.getTV()) {
            double dy = LIMELIGHT_BACK_POSITION.getZ() * LIMELIGHT_BACK_PITCH.plus(Rotation2d.fromDegrees(BACK_LIMELIGHT.getTY())).getTan();
            double dx = dy * Rotation2d.fromDegrees(BACK_LIMELIGHT.getTX()).getTan();

            Pose2d ePose = drivetrainInputs.estimatedPose;

            double nx = ePose.getX() - dy * ePose.getRotation().getCos() - dx * ePose.getRotation().getSin();
            double ny = ePose.getY() - dy * ePose.getRotation().getSin() + dx * ePose.getRotation().getCos();

            drivetrainInputs.notePose = new Pose2d(nx, ny, new Rotation2d(0));
        }

        gyroscope.log(subdirectory + "/" + humanReadableName, "Pigeon");

        Logger.processInputs(subdirectory + "/" + humanReadableName, drivetrainInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Drivetrain");
        updateOdometry();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for(int i = 0; i < numModules; i++) {
            states[i] = modules[i].getState();
        }

        return states;
        
    }

    public void resetModules() {
        for(SwerveModule module : modules) {
            module.resetModule();
        }
    }
}
