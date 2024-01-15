package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Loggable;
import frc.robot.utils.Vortex;
import frc.robot.utils.VortexBuilder;

import static frc.robot.constants.HardwareMap.*;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    @AutoLog
    public static class IntakeInputs {

    }
    
    private IntakeInputsAutoLogged intakeInputs;
    private Vortex intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = VortexBuilder.createWithDefaults(INTAKE_MOTOR_ID)
            .build();

        intakeInputs = new IntakeInputsAutoLogged();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        
        intakeMotor.log(subdirectory + "/" + humanReadableName, "IntakeMotor");

        Logger.processInputs(subdirectory + "/" + humanReadableName, intakeInputs);
    }

    public void runMotor(double power) {
        intakeMotor.set(power);
    }

    public void stopMotor() {
        intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        log("Subsystems", "IntakeSubsystem");

    }
}
