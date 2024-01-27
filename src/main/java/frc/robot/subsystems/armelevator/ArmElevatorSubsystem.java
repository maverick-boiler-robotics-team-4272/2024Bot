package frc.robot.subsystems.armelevator;

import static frc.robot.constants.HardwareMap.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.*;
public class ArmElevatorSubsystem extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ArmInputs {

    }

    private Vortex elevatorMotor1; // Might become a NEO, not yet decided by CAD
    private Vortex elevatorMotor2; // Might become a NEO, not yet decided by CAD
    private Vortex armMotor;

    private ArmInputsAutoLogged armInputs;

    public ArmElevatorSubsystem() {
        elevatorMotor1 = VortexBuilder.createWithDefaults(ELEVATOR_MOTOR_1_ID)
            .build();
        elevatorMotor2 = VortexBuilder.createWithDefaults(ELEVATOR_MOTOR_2_ID)
            .build();
        armMotor = VortexBuilder.createWithDefaults(ARM_MOTOR_ID)
            .build();

        armInputs = new ArmInputsAutoLogged();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        elevatorMotor1.log(subdirectory + "/" + humanReadableName, "ElevatorMotor1");
        elevatorMotor2.log(subdirectory + "/" + humanReadableName, "ElevatorMotor2");
        armMotor.log(subdirectory + "/" + humanReadableName, "ArmMotor");

        Logger.processInputs(subdirectory + "/" + humanReadableName, armInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "ArmElevator");
    }
}
