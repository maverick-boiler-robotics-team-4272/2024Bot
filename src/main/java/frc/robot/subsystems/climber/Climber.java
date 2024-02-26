package frc.robot.subsystems.climber;


// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;

// Hardware
import frc.robot.utils.hardware.*;

// Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import static frc.robot.constants.HardwareMap.*;

public class Climber extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ClimberInputs {

    }

    private Vortex climberMotor;
    private ClimberInputsAutoLogged climberInputs;

    public Climber() {
        climberMotor = VortexBuilder.createWithDefaults(CLIMBER_MOTOR_1_ID)
            .withCurrentLimit(60)
            .withAllPeriodicFramerates(65535)
            .build();

        climberInputs =  new ClimberInputsAutoLogged();
    }

    public void runMotor(double power) {
        climberMotor.set(power);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        climberMotor.log(subdirectory + "/" + humanReadableName, "ClimbMotor");

        Logger.processInputs(subdirectory + "/" + humanReadableName, climberInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Climber");
    }
}
