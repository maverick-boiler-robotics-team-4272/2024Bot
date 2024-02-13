package frc.robot.subsystems.armelevator;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.ArmConstants.*;
import static frc.robot.constants.RobotConstants.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ArmElevatorSetpoint;
import frc.robot.utils.*;
public class ArmElevatorSubsystem extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ArmElevatorInputs {
        public double currentArmAngleRadians;
        public double desiredArmAngleRadians;
        public double safeArmAngleRadians;

        public double currentElevatorHeight;
        public double desiredElevatorHeight;
        public double safeElevatorHeight;
    }

    private NEO elevatorMotor1;
    private NEO elevatorMotor2;
    private Vortex armMotor;

    private RelativeEncoder armEncoder;
    private RelativeEncoder elevatorEncoder;
    
    private SparkPIDController elevatorController;
    private SparkPIDController armController;

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ARM_PID_F, 0, 0);

    private ArmElevatorInputsAutoLogged armElevatorInputs;

    private Rotation2d desiredArmAngle;
    private double desiredElevatorHeight;

    public ArmElevatorSubsystem() {
        elevatorMotor1 = NEOBuilder.createWithDefaults(ELEVATOR_MOTOR_1_ID)
            .withPosition(0)
            .withPositionConversionFactor(ELEVATOR_RATIO)
            .withSoftLimits(MAX_ELEVATOR_HEIGHT, MIN_ELEVATOR_HEIGHT)
            .withPIDParams(ELEVATOR_PID_P, ELEVATOR_PID_I, ELEVATOR_PID_D)
            .build();
        elevatorMotor2 = NEOBuilder.createWithDefaults(ELEVATOR_MOTOR_2_ID)
            .asFollower(elevatorMotor1, false)
            .build();
        armMotor = VortexBuilder.createWithDefaults(ARM_MOTOR_ID)
            .withPositionConversionFactor(ARM_RATIO)
            .withSoftLimits(MAX_ARM_ANGLE.getRadians(), MIN_ARM_ANGLE.getRadians())
            .withPIDParams(ARM_PID_P, ARM_PID_I, ARM_PID_D)
            .build();

        armElevatorInputs = new ArmElevatorInputsAutoLogged();

        armController = armMotor.getPIDController();
        elevatorController = elevatorMotor1.getPIDController();

        armEncoder = armMotor.getEncoder();
        elevatorEncoder = elevatorMotor1.getEncoder();
    }

    private void setShooterRotation(Rotation2d r) {
        armController.setReference(r.getRadians(), ControlType.kPosition, 0, armFeedforward.calculate(armEncoder.getPosition(), 0));
    }

    private void setElevatorHeight(double h) {
        elevatorController.setReference(h, ControlType.kPosition, 0, ELEVATOR_PID_F);
    }

    public boolean isAtPosition() {
        return Math.abs(desiredElevatorHeight - elevatorEncoder.getPosition()) < ELEVATOR_HEIGHT_DEADZONE && 
            Math.abs(desiredArmAngle.getRadians() - armEncoder.getPosition()) < ARM_ANGLE_DEADZONE.getRadians();
    }

    public void goToPos(Rotation2d r, double h) {
        desiredArmAngle = r;
        desiredElevatorHeight = h;

        armElevatorInputs.desiredArmAngleRadians = r.getRadians();
        armElevatorInputs.desiredElevatorHeight = h;
    }

    public void goToPos(ArmElevatorSetpoint setpoint) {
        goToPos(setpoint.getArmAngle(), setpoint.getElevatorHeight());
    }

    public void targetPos(Translation2d drivetrainPos, Translation3d position) {
        double height = elevatorEncoder.getPosition();

        double d = Math.hypot(drivetrainPos.getX() - position.getX(), drivetrainPos.getY() - position.getY()) - ELEVATOR_TRANSLATION.getY();

        Rotation2d theta = new Rotation2d(position.getZ() - height + ELEVATOR_TRANSLATION.getZ(), d);
        

        if(theta.getDegrees() > MAX_SAFE_ANGLE.getDegrees()) {
            height = position.getZ() - d * MAX_SAFE_ANGLE.getTan();
            theta = MAX_SAFE_ANGLE;
        }

        goToPos(theta, height);
    }

    private void handleSaftey() {
        //LOGIC
        double elevatorHeight = elevatorEncoder.getPosition();

        double safeTheta = Math.acos(Math.min(1, Math.max((elevatorHeight - BLOCKING_HEIGHT) / ARM_LENGTH, -1.0)));
        double safeHeight = ARM_LENGTH * desiredArmAngle.getCos() + BLOCKING_HEIGHT;

        // Height safe, angle safe
        // Height safe, angle  not
        // Height  not, angle safe
        // Height not, angle not

        if(desiredArmAngle.getRadians() > safeTheta) {
            setShooterRotation(desiredArmAngle);
        } else {
            setShooterRotation(new Rotation2d(safeTheta));
        }

        if(desiredElevatorHeight > safeHeight) {
            setElevatorHeight(desiredElevatorHeight);
        } else {
            setElevatorHeight(safeHeight);
        }

        armElevatorInputs.safeArmAngleRadians = safeTheta;
        armElevatorInputs.safeElevatorHeight = safeHeight;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        elevatorMotor1.log(subdirectory + "/" + humanReadableName, "ElevatorMotor1");
        elevatorMotor2.log(subdirectory + "/" + humanReadableName, "ElevatorMotor2");
        armMotor.log(subdirectory + "/" + humanReadableName, "ArmMotor");

        armElevatorInputs.currentElevatorHeight = elevatorEncoder.getPosition();
        armElevatorInputs.currentArmAngleRadians = armEncoder.getPosition();

        Logger.processInputs(subdirectory + "/" + humanReadableName, armElevatorInputs);
    }

    @Override
    public void periodic() {
        handleSaftey();

        log("Subsystems", "ArmElevator");
    }
}
