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

    private Vortex climberMotor1;
    private Vortex climberMotor2;
    private ClimberInputsAutoLogged climberInputs;

    public Climber() {
        climberMotor1 = VortexBuilder.createWithDefaults(CLIMBER_MOTOR_1_ID)
            .build();

        climberMotor2 = VortexBuilder.createWithDefaults(CLIMBER_MOTOR_2_ID)
            .build();

        climberInputs =  new ClimberInputsAutoLogged();
    }

    public void runMotors(double motor1Power, double motor2Power) {
        climberMotor1.set(motor1Power);
        climberMotor2.set(motor2Power);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        climberMotor1.log(subdirectory + "/" + humanReadableName, "ClimbMotor1");
        climberMotor2.log(subdirectory + "/" + humanReadableName, "ClimbMotor2");

        Logger.processInputs(subdirectory + "/" + humanReadableName, climberInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Climber");
    }
}
