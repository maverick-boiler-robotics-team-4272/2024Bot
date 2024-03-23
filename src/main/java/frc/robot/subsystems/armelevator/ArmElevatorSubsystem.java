package frc.robot.subsystems.armelevator;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;
import frc.robot.utils.misc.InterpolationMap;
import frc.team4272.globals.MathUtils;
// Hardware
import frc.robot.utils.hardware.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.Pair;
// Math
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.*;

// Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import frc.robot.constants.RobotConstants.ArmElevatorSetpoint;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.NOMINAL_VOLTAGE;
import static frc.robot.constants.RobotConstants.ArmConstants.*;
import static frc.robot.constants.RobotConstants.ElevatorConstants.*;

public class ArmElevatorSubsystem extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ArmElevatorInputs {
        public double currentArmAngleRadians;
        public double desiredArmAngleRadians;
        public double safeArmAngleRadians;
        public double armAngleErrorRadians;

        public double currentElevatorHeight;
        public double desiredElevatorHeight;
        public double safeElevatorHeight;
        public double elevatorHeightError;
    }

    private NEO elevatorMotor1;
    private NEO elevatorMotor2;
    private Vortex armMotor;

    private RelativeEncoder armEncoder;
    private RelativeEncoder elevatorEncoder;

    private MAVCoder2 armAbsoluteEncoder;
    
    private SparkPIDController elevatorController;
    private SparkPIDController armController;

    @SuppressWarnings({ "unchecked" })
    private InterpolationMap map = new InterpolationMap((Pair<Double, Double>[])new Pair[] {
        new Pair<Double, Double>(1.67, 45.0),
        new Pair<Double, Double>(2.20, 40.0),
        new Pair<Double, Double>(2.65, 37.0),
        new Pair<Double, Double>(3.37, 33.5),
        new Pair<Double, Double>(4.73, 30.0)
    });

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ARM_PID_F, 0, 0);

    private ArmElevatorInputsAutoLogged armElevatorInputs;

    private Rotation2d desiredArmAngle;

    public ArmElevatorSubsystem() {
        elevatorMotor1 = NEOBuilder.create(ELEVATOR_MOTOR_1_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withPosition(0)
            .withPositionConversionFactor(ELEVATOR_RATIO)
            .withSoftLimits(MAX_ELEVATOR_HEIGHT, MIN_ELEVATOR_HEIGHT)
            .withPIDParams(ELEVATOR_PID_P, ELEVATOR_PID_I, ELEVATOR_PID_D)
            .withPIDClamping(ELEVATOR_OUTPUT_MIN, ELEVATOR_OUTPUT_MAX)
            .withInversion(true)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withCurrentLimit(50)
            .build();
        
        elevatorMotor2 = NEOBuilder.create(ELEVATOR_MOTOR_2_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withIdleMode(IdleMode.kBrake)
            .asFollower(elevatorMotor1, true)
            .withCurrentLimit(50)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus2, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .getUnburntNeo();

        armMotor = VortexBuilder.create(ARM_MOTOR_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .withPositionConversionFactor(ARM_RATIO)
            .withCurrentLimit(40)
            // .withPosition(0.0)
            .withSoftLimits(MAX_ARM_ANGLE.getRadians(), MIN_ARM_ANGLE.getRadians())
            .withPIDParams(ARM_PID_P, ARM_PID_I, ARM_PID_D)
            .withPIDPositionWrapping(0, 2 * Math.PI)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .getUnburntNeo();

        armElevatorInputs = new ArmElevatorInputsAutoLogged();

        armElevatorInputs.currentArmAngleRadians = 0;
        armElevatorInputs.desiredArmAngleRadians = 0;
        armElevatorInputs.desiredElevatorHeight = 0;
        armElevatorInputs.currentElevatorHeight = 0;

        desiredArmAngle = Rotation2d.fromRadians(armElevatorInputs.desiredArmAngleRadians);

        armAbsoluteEncoder = new MAVCoder2(elevatorMotor2, ARM_OFFSET);
        elevatorMotor2.burnFlash();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {}
        
        System.out.println(String.format("ArmElevatorPos: %.2f", armAbsoluteEncoder.getUnoffsetPosition()));

        armController = armMotor.getPIDController();
        elevatorController = elevatorMotor1.getPIDController();

        armEncoder = armMotor.getEncoder();
        elevatorEncoder = elevatorMotor1.getEncoder();

        armEncoder.setPosition(-armAbsoluteEncoder.getPosition() * Math.PI / 180.0);
    }

    public void resetArmMotor() {
        armMotor.setInverted(false);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(40);
        // armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)MAX_ARM_ANGLE.getRadians());
        // armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)MIN_ARM_ANGLE.getRadians());

        double position = armAbsoluteEncoder.getPosition();

        armEncoder.setPosition(-MathUtils.inputModulo(position, -180, 180) * Math.PI / 180.0);
    }

    private void setShooterRotation(Rotation2d r) {
        armController.setReference(r.getRadians(), ControlType.kPosition, 0, armFeedforward.calculate(armEncoder.getPosition(), 0));
    }

    private void setElevatorHeight(double h) {
        elevatorController.setReference(h, ControlType.kPosition, 0, ELEVATOR_PID_F);
    }

    public boolean isAtPosition() {
        return Math.abs(armElevatorInputs.desiredElevatorHeight - elevatorEncoder.getPosition()) < ELEVATOR_HEIGHT_DEADZONE && 
            Math.abs(desiredArmAngle.getRadians() - armEncoder.getPosition()) < ARM_ANGLE_DEADZONE.getRadians();
    }

    public void goToPos(Rotation2d r, double h) {
        desiredArmAngle = r;

        armElevatorInputs.desiredArmAngleRadians = r.getRadians();
        armElevatorInputs.desiredElevatorHeight = h;
    }

    public void goToPos(ArmElevatorSetpoint setpoint) {
        goToPos(setpoint.getArmAngle(), setpoint.getElevatorHeight());
    }

    public void targetPos(Translation2d drivetrainPos, Translation3d position) {
        Rotation2d theta = Rotation2d.fromDegrees(map.getInterpolatedValue(drivetrainPos.getDistance(position.toTranslation2d())));
        
        if(theta.getDegrees() > MAX_SAFE_ANGLE.getDegrees()) {
            theta = MAX_SAFE_ANGLE;
        } else if(theta.getDegrees() < MIN_SAFE_ANGLE.getDegrees()) {
            theta = MIN_SAFE_ANGLE;
        }

        goToPos(theta, 0.0);
    }

    public void runElevator(double power) {
        elevatorMotor1.set(power);
    }

    public void zeroElevator() {
        elevatorEncoder.setPosition(0);
    }

    public void elevatorGoNyrooom() {
        elevatorMotor1.setSmartCurrentLimit(80);
        elevatorMotor2.setSmartCurrentLimit(80);
    }

    public void elevatorGoNotSoNyroom() {
        elevatorMotor1.setSmartCurrentLimit(50);
        elevatorMotor2.setSmartCurrentLimit(50);
    }

    public void handleSafety() {
        double elevatorHeight = elevatorEncoder.getPosition();

        double safeTheta = Math.acos(Math.min(1, Math.max((elevatorHeight + ELEVATOR_TRANSLATION.getZ() - BLOCKING_HEIGHT) / ARM_LENGTH, -1.0)));
        double safeHeight = -ARM_LENGTH * desiredArmAngle.getSin() + BLOCKING_HEIGHT;

        // Height safe, angle safe
        // Height safe, angle  not
        // Height  not, angle safe
        // Height  not, angle not

        if(Math.abs(desiredArmAngle.getRadians() + Math.PI / 2.0) > Math.abs(safeTheta)) {
            setShooterRotation(desiredArmAngle);
        } else {
            setShooterRotation(new Rotation2d(safeTheta + Math.PI / 2.0));
        }

        if(armElevatorInputs.desiredElevatorHeight > safeHeight) {
            setElevatorHeight(armElevatorInputs.desiredElevatorHeight);
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
        armAbsoluteEncoder.log(subdirectory + "/" + humanReadableName, "ArmAbsoluteEncoder");

        armElevatorInputs.currentElevatorHeight = elevatorEncoder.getPosition();
        armElevatorInputs.currentArmAngleRadians = armEncoder.getPosition();

        armElevatorInputs.armAngleErrorRadians = armElevatorInputs.desiredArmAngleRadians - armElevatorInputs.currentArmAngleRadians;
        armElevatorInputs.elevatorHeightError = armElevatorInputs.desiredElevatorHeight - armElevatorInputs.currentElevatorHeight;

        Logger.processInputs(subdirectory + "/" + humanReadableName, armElevatorInputs);
    }

    @Override
    public void periodic() {
        handleSafety();
        log("Subsystems", "ArmElevator");
    }
}
