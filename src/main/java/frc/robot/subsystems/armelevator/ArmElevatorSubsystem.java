package frc.robot.subsystems.armelevator;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;

// Hardware
import frc.robot.utils.hardware.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

// Math
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.*;

// Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import frc.robot.constants.RobotConstants.ArmElevatorSetpoint;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.ArmConstants.*;
import static frc.robot.constants.RobotConstants.ElevatorConstants.*;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;

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

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ARM_PID_F, 0, 0);

    private ArmElevatorInputsAutoLogged armElevatorInputs;

    private Rotation2d desiredArmAngle;

    public ArmElevatorSubsystem() {
        elevatorMotor1 = NEOBuilder.createWithDefaults(ELEVATOR_MOTOR_1_ID)
            .withPosition(0)
            .withPositionConversionFactor(ELEVATOR_RATIO)
            .withSoftLimits(MAX_ELEVATOR_HEIGHT, MIN_ELEVATOR_HEIGHT)
            .withPIDParams(ELEVATOR_PID_P, ELEVATOR_PID_I, ELEVATOR_PID_D)
            .withInversion(true)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withCurrentLimit(50)
            .build();
        
        elevatorMotor2 = NEOBuilder.createWithDefaults(ELEVATOR_MOTOR_2_ID)
            .asFollower(elevatorMotor1, true)
            .withCurrentLimit(50)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus2, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .getUnburntNeo();
        armMotor = VortexBuilder.createWithDefaults(ARM_MOTOR_ID)
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

    private void setShooterRotation(Rotation2d r) {
        armController.setReference(r.getRadians(), ControlType.kPosition, 0, armFeedforward.calculate(armEncoder.getPosition(), 0));
    }

    private void setElevatorHeight(double h) {
        elevatorController.setReference(h, ControlType.kPosition, 0, TESTING_TABLE.getNumber("Elevator PID F"));
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
        double height = elevatorEncoder.getPosition();

        double distFromSpeaker = Math.hypot(drivetrainPos.getX() - position.getX(), drivetrainPos.getY() - position.getY()) + ELEVATOR_TRANSLATION.getY();

        Rotation2d theta = new Rotation2d(distFromSpeaker, position.getZ() - (height + ELEVATOR_TRANSLATION.getZ()));
        

        if(theta.getDegrees() > MAX_SAFE_ANGLE.getDegrees()) {
            theta = MAX_SAFE_ANGLE;
        }

        goToPos(theta, 0.0);
    }

    public void runElevator(double power) {
        elevatorMotor1.set(power);
    }

    public void zeroElevator() {
        elevatorEncoder.setPosition(0);
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
