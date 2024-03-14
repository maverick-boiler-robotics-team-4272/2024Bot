package frc.robot.subsystems.intake;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;

// Hardware
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.utils.hardware.*;

// Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.NOMINAL_VOLTAGE;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    @AutoLog
    public static class IntakeInputs {

    }
    
    private IntakeInputsAutoLogged intakeInputs;
    private Vortex intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = VortexBuilder.create(INTAKE_MOTOR_1_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withInversion(false)
            .withCurrentLimit(80)
            .withIdleMode(IdleMode.kCoast)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus2, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus5, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus6, 500)
            .build();

        try {
            Thread.sleep(200);
        } catch(InterruptedException e) {

        }

        intakeInputs = new IntakeInputsAutoLogged();
    }

    public void resetIntakeMotor() {
        intakeMotor.setInverted(false);
        intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.setIdleMode(IdleMode.kCoast);
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

    public void setIntakeCurrentLimit() {
        intakeMotor.setSmartCurrentLimit(80);
    }

    public boolean isMotorStalling() {
        return intakeMotor.isStalling();
    }

    @Override
    public void periodic() {
        log("Subsystems", "IntakeSubsystem");

    }
}
