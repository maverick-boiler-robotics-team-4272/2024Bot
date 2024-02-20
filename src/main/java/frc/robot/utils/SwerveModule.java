package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.team4272.swerve.utils.SwerveModuleBase;

import static frc.robot.constants.RobotConstants.DrivetrainConstants.SwerveModuleConstants.*;
import static frc.robot.constants.UniversalConstants.*;

public class SwerveModule extends SwerveModuleBase implements Loggable {
    
    @AutoLog
    public static class SwerveModuleInputs {
        public double currentMotorAngleDegrees;
        public double currentSpeedMetersPerSecond;
        public double desiredMotorAngleDegrees;
        public double desiredSpeedMetersPerSecond;

        public double speedError;
        public double currentDesiredSpeedRatio;
    }

    private NEO driveMotor;
    private SparkPIDController drivePidController;
    private RelativeEncoder driveEncoder;

    private NEO steerMotor;
    private SparkPIDController steerPidController;
    private RelativeEncoder steerEncoder;

    private MAVCoder2 externalEncoder;
    private SwerveModuleInputsAutoLogged moduleInputs;

    public SwerveModule(int id, double offset) {
        // driveMotor = new NEO(id);
        driveMotor = NEOBuilder.createWithDefaults(id)
            .withCurrentLimit(40)
            .withPIDPositionWrapping(-180, 180)
            .withVelocityConversionFactor(WHEEL_RADIUS * PI2 / (60.0 * DRIVE_RATIO * Units.Inches.convertFrom(1.0, Units.Meters)))
            .withPositionConversionFactor(WHEEL_RADIUS * PI2 / (DRIVE_RATIO * Units.Inches.convertFrom(1.0, Units.Meters)))
            .withPosition(0)
            .withPIDFParams(DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D, DRIVE_PID_F)
            .build();
        drivePidController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();

        steerMotor = NEOBuilder.createWithDefaults(id + 10)
            .withCurrentLimit(40)
            .withPositionConversionFactor(360.0 / STEER_RATIO)
            .withPIDFParams(STEER_PID_P, STEER_PID_I, STEER_PID_D, STEER_PID_F)
            .withPIDPositionWrapping(-180, 180)
            .getUnburntNeo();
        steerPidController = steerMotor.getPIDController();

        steerEncoder = steerMotor.getEncoder();

        externalEncoder = new MAVCoder2(steerMotor, offset);

        steerMotor.burnFlash();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            
        }

        System.out.println(externalEncoder.getUnoffsetPosition());

        steerEncoder.setPosition(-externalEncoder.getPosition());
        // steerEncoder.setPosition(0);
        moduleInputs = new SwerveModuleInputsAutoLogged();

    }

    public void setCoastMode(boolean mode) {
        driveMotor.setIdleMode(mode ? IdleMode.kCoast : IdleMode.kBrake);
    }

    public Rotation2d getMotorRotation() {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }

    public Rotation2d getExternalEncoderRotation() {
        return Rotation2d.fromDegrees(externalEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getMotorRotation());
    }

    @Override
    public void goToState(SwerveModuleState state) {
        SwerveModuleState optimized = optimize(state, getMotorRotation());

        moduleInputs.desiredMotorAngleDegrees = optimized.angle.getDegrees();
        moduleInputs.desiredSpeedMetersPerSecond = optimized.speedMetersPerSecond;
        
        drivePidController.setReference(moduleInputs.desiredSpeedMetersPerSecond, ControlType.kVelocity);
        steerPidController.setReference(moduleInputs.desiredMotorAngleDegrees, ControlType.kPosition);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(180).minus(getMotorRotation()));
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        moduleInputs.currentMotorAngleDegrees = steerEncoder.getPosition();
        moduleInputs.currentSpeedMetersPerSecond = driveEncoder.getVelocity();

        moduleInputs.speedError = moduleInputs.desiredSpeedMetersPerSecond - moduleInputs.currentSpeedMetersPerSecond;
        moduleInputs.currentDesiredSpeedRatio = moduleInputs.desiredSpeedMetersPerSecond / moduleInputs.currentSpeedMetersPerSecond;

        driveMotor.log(subdirectory + "/" + humanReadableName, "DriveMotor");
        steerMotor.log(subdirectory + "/" + humanReadableName, "SteerMotor");
        driveMotor.log(subdirectory + "/" + humanReadableName, "MAVCoder");

        Logger.processInputs(subdirectory + "/" + humanReadableName, moduleInputs);
    }
}
