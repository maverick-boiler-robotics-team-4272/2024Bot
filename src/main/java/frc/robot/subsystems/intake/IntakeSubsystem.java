package frc.robot.subsystems.intake;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;

// Hardware
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.utils.hardware.*;

// Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import static frc.robot.constants.HardwareMap.*;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    @AutoLog
    public static class IntakeInputs {

    }
    
    private IntakeInputsAutoLogged intakeInputs;
    private Vortex intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = VortexBuilder.createWithDefaults(INTAKE_MOTOR_1_ID)
            .withIdleMode(IdleMode.kCoast)
            .withCurrentLimit(80)
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
