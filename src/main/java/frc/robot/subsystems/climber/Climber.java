package frc.robot.subsystems.climber;


// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;


// Hardware
import frc.robot.utils.hardware.*;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

// Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.ClimberConstants.*;

public class Climber extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ClimberInputs {
        public double climberDesiredHeight;
    }

    private Vortex climberMotor;
    private SparkPIDController climberController;
    private ClimberInputsAutoLogged climberInputs;

    public Climber() {
        climberMotor = VortexBuilder.createWithDefaults(CLIMBER_MOTOR_1_ID)
            .withCurrentLimit(20)
            .withPosition(0.0)
            .withPIDParams(CLIMBER_P, CLIMBER_I, CLIMBER_D)
            .withSoftLimits(CLIMBER_MAX_HEIGHT, CLIMBER_MIN_HEIGHT)
            // .withInversion(true)
            // .withAllPeriodicFramerates(65535)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .build();

        try {
            Thread.sleep(100);
        } catch(InterruptedException e) {

        }

        climberController = climberMotor.getPIDController();

        climberInputs =  new ClimberInputsAutoLogged();
    }

    public void setClimerHeight(double height) {
        if(height != climberInputs.climberDesiredHeight)
        climberController.setReference(height, ControlType.kPosition, 0, CLIMBER_F);
        climberInputs.climberDesiredHeight = height;
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
