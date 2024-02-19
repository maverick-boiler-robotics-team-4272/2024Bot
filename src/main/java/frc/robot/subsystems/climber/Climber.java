package frc.robot.subsystems.climber;

import static frc.robot.constants.HardwareMap.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;

public class Climber extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ClimberInputs {

    }

    private Vortex climberMotor;
    private ClimberInputsAutoLogged climberInputs;

    public Climber() {
        climberMotor = VortexBuilder.createWithDefaults(CLIMBER_MOTOR_1_ID)
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
