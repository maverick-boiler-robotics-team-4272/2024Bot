package frc.robot.subsystems.armelevator;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.ArmConstants.*;
import static frc.robot.constants.RobotConstants.ElevatorConstants.*;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.TESTING_TABLE;
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
        public double armAngleErrorRadians;
        public double safeArmAngleRadians;

        public double currentElevatorHeight;
        public double desiredElevatorHeight;
        public double elevatorHeightError;
        public double safeElevatorHeight;
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
    private double desiredElevatorHeight;

    public ArmElevatorSubsystem() {
        elevatorMotor1 = NEOBuilder.createWithDefaults(ELEVATOR_MOTOR_1_ID)
            .withPosition(0)
            .withPositionConversionFactor(ELEVATOR_RATIO)
            .withSoftLimits(MAX_ELEVATOR_HEIGHT, MIN_ELEVATOR_HEIGHT)
            .withPIDParams(ELEVATOR_PID_P, ELEVATOR_PID_I, ELEVATOR_PID_D)
            // .withPIDOutputClamping(-1.0, 2.0)
            .withInversion(true)
            .withCurrentLimit(50)
            .build();
        elevatorMotor2 = NEOBuilder.createWithDefaults(ELEVATOR_MOTOR_2_ID)
            .asFollower(elevatorMotor1, true)
            .withCurrentLimit(50)
            .getUnburntNeo();
        armMotor = VortexBuilder.createWithDefaults(ARM_MOTOR_ID)
            .withPositionConversionFactor(ARM_RATIO)
            .withCurrentLimit(40)
            .withPosition(0.0)
            .withSoftLimits(MAX_ARM_ANGLE.getRadians(), MIN_ARM_ANGLE.getRadians())
            .withPIDParams(ARM_PID_P, ARM_PID_I, ARM_PID_D)
            .build();

        armElevatorInputs = new ArmElevatorInputsAutoLogged();

        armElevatorInputs.currentArmAngleRadians = 0;
        armElevatorInputs.desiredArmAngleRadians = 0;
        armElevatorInputs.safeArmAngleRadians = 0;
        armElevatorInputs.desiredElevatorHeight = 0;
        armElevatorInputs.currentElevatorHeight = 0;
        armElevatorInputs.safeElevatorHeight = 0;

        desiredArmAngle = Rotation2d.fromRadians(armElevatorInputs.desiredArmAngleRadians);
        desiredElevatorHeight = 0;

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

        TESTING_TABLE.putNumber("Elevator PID P", ELEVATOR_PID_P);
        TESTING_TABLE.putNumber("Elevator PID I", ELEVATOR_PID_I);
        TESTING_TABLE.putNumber("Elevator PID D", ELEVATOR_PID_D);
        TESTING_TABLE.putNumber("Elevator PID F", ELEVATOR_PID_F);

    }

    private void setShooterRotation(Rotation2d r) {
        armController.setReference(r.getRadians(), ControlType.kPosition, 0, armFeedforward.calculate(armEncoder.getPosition(), 0));
    }

    private void setElevatorHeight(double h) {
        elevatorController.setReference(h, ControlType.kPosition, 0, TESTING_TABLE.getNumber("Elevator PID F"));
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

        double distFromSpeaker = Math.hypot(drivetrainPos.getX() - position.getX(), drivetrainPos.getY() - position.getY()) - ELEVATOR_TRANSLATION.getY();

        Rotation2d theta = new Rotation2d(distFromSpeaker, position.getZ() - (height + ELEVATOR_TRANSLATION.getZ()));
        

        if(theta.getDegrees() > MAX_SAFE_ANGLE.getDegrees()) {
            // height = position.getZ() - distFromSpeaker * MAX_SAFE_ANGLE.getTan();
            theta = MAX_SAFE_ANGLE;
        }

        goToPos(theta, 0.0);
    }

    private void handleSaftey() {
        //LOGIC
        double elevatorHeight = elevatorEncoder.getPosition();

        double safeTheta = Math.acos(Math.min(1, Math.max((elevatorHeight - BLOCKING_HEIGHT) / ARM_LENGTH, -1.0)));
        double safeHeight = ARM_LENGTH * desiredArmAngle.getCos() + BLOCKING_HEIGHT;

        // Height safe, angle safe
        // Height safe, angle  not
        // Height  not, angle safe
        // Height  not, angle not

        // if(Math.abs(desiredArmAngle.getRadians() + Math.PI / 2.0) > Math.abs(safeTheta)) {
        //     setShooterRotation(desiredArmAngle);
        // } else {
        //     setShooterRotation(new Rotation2d(safeTheta + Math.PI / 2.0));
        // }

        // if(desiredElevatorHeight > safeHeight) {
        //     setElevatorHeight(desiredElevatorHeight);
        // } else {
        //     setElevatorHeight(safeHeight);
        // }

        setElevatorHeight(desiredElevatorHeight);
        setShooterRotation(desiredArmAngle);

        armElevatorInputs.safeArmAngleRadians = safeTheta;
        armElevatorInputs.safeElevatorHeight = safeHeight;
    }

    public void runElevator(double power) {
        elevatorMotor1.set(power);
    }

    public void zeroElevator() {
        elevatorEncoder.setPosition(0);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        elevatorMotor1.log(subdirectory + "/" + humanReadableName, "ElevatorMotor1");
        elevatorMotor2.log(subdirectory + "/" + humanReadableName, "ElevatorMotor2");
        armMotor.log(subdirectory + "/" + humanReadableName, "ArmMotor");

        armElevatorInputs.currentElevatorHeight = elevatorEncoder.getPosition();
        armElevatorInputs.currentArmAngleRadians = armEncoder.getPosition();

        armElevatorInputs.armAngleErrorRadians = armElevatorInputs.desiredArmAngleRadians - armElevatorInputs.currentArmAngleRadians;
        armElevatorInputs.elevatorHeightError = armElevatorInputs.desiredElevatorHeight - armElevatorInputs.currentElevatorHeight;

        Logger.processInputs(subdirectory + "/" + humanReadableName, armElevatorInputs);
    }

    @Override
    public void periodic() {
        handleSaftey();

        log("Subsystems", "ArmElevator");

        elevatorController.setP(TESTING_TABLE.getNumber("Elevator PID P"));
        elevatorController.setI(TESTING_TABLE.getNumber("Elevator PID I"));
        elevatorController.setD(TESTING_TABLE.getNumber("Elevator PID D"));
        elevatorController.setFF(TESTING_TABLE.getNumber("Elevator PID F"));

    }
}
