package frc.robot.utils.hardware;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.Loggable;

// Spark Hardware
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

// Math
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

// Swerve Software
import frc.team4272.swerve.utils.SwerveModuleBase;

// Constants
import static frc.robot.constants.RobotConstants.DrivetrainConstants.SwerveModuleConstants.*;
import static frc.robot.constants.UniversalConstants.*;

public class SwerveModule extends SwerveModuleBase implements Loggable {
    
    @AutoLog
    public static class SwerveModuleInputs {
        public SwerveModuleState currentState;
        public SwerveModuleState desiredState;

        public double currentMotorAngleDegrees;
        public double currentSpeedMetersPerSecond;
        public double desiredMotorAngleDegrees;
        public double desiredSpeedMetersPerSecond;
    }

    private NEO driveMotor;
    private SparkPIDController drivePidController;
    private RelativeEncoder driveEncoder;

    private NEO steerMotor;
    private SparkPIDController steerPidController;
    private RelativeEncoder steerEncoder;

    private MAVCoder2 externalEncoder;
    private SwerveModuleInputsAutoLogged moduleInputs;

    private SwerveModulePosition previousPosition = new SwerveModulePosition();

    public SwerveModule(int id, double offset) {
        // driveMotor = new NEO(id);
        driveMotor = NEOBuilder.createWithDefaults(id)
            .withCurrentLimit(40)
            .withPIDPositionWrapping(-180, 180)
            .withVelocityConversionFactor(WHEEL_RADIUS * PI2 / (60.0 * DRIVE_RATIO * Units.Inches.convertFrom(1.0, Units.Meters)))
            .withPositionConversionFactor(WHEEL_RADIUS * PI2 / (DRIVE_RATIO * Units.Inches.convertFrom(1.0, Units.Meters)))
            .withPosition(0)
            .withPIDFParams(DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D, DRIVE_PID_F)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .build();
        drivePidController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();

        steerMotor = NEOBuilder.createWithDefaults(id + 10)
            .withCurrentLimit(40)
            .withPositionConversionFactor(360.0 / STEER_RATIO)
            .withPIDFParams(STEER_PID_P, STEER_PID_I, STEER_PID_D, STEER_PID_F)
            .withMaxIAccum(STEER_PID_I_MAX)
            .withPIDPositionWrapping(-180, 180)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .getUnburntNeo();
        steerPidController = steerMotor.getPIDController();

        steerEncoder = steerMotor.getEncoder();

        externalEncoder = new MAVCoder2(steerMotor, offset);

        steerMotor.burnFlash();
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            
        }

        System.out.println(externalEncoder.getUnoffsetPosition());

        steerEncoder.setPosition(-externalEncoder.getPosition());
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
        moduleInputs.currentState = new SwerveModuleState(driveEncoder.getVelocity(), getMotorRotation());
        return new SwerveModuleState(moduleInputs.currentState.speedMetersPerSecond, moduleInputs.currentState.angle);
    }

    public void resetModule() {
        steerEncoder.setPosition(-externalEncoder.getPosition());
    }

    @Override
    public void goToState(SwerveModuleState state) {
        moduleInputs.desiredState = state;

        SwerveModuleState optimized = optimize(state, getMotorRotation());
        
        drivePidController.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);
        steerPidController.setReference(optimized.angle.getDegrees(), ControlType.kPosition);
    }

    @Override
    public SwerveModulePosition getPosition() {
        double drivePosition = driveEncoder.getPosition();
        if(Math.abs(drivePosition - previousPosition.distanceMeters) > 1)
            return previousPosition;

        previousPosition = new SwerveModulePosition(drivePosition, Rotation2d.fromDegrees(180).minus(getMotorRotation()));
        return previousPosition;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        moduleInputs.currentMotorAngleDegrees = steerEncoder.getPosition();
        moduleInputs.currentSpeedMetersPerSecond = driveEncoder.getVelocity();

        driveMotor.log(subdirectory + "/" + humanReadableName, "DriveMotor");
        steerMotor.log(subdirectory + "/" + humanReadableName, "SteerMotor");
        externalEncoder.log(subdirectory + "/" + humanReadableName, "MAVCoder");

        Logger.processInputs(subdirectory + "/" + humanReadableName, moduleInputs);
    }
}
