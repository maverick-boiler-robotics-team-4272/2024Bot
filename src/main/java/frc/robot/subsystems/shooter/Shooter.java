package frc.robot.subsystems.shooter;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;

import com.revrobotics.CANSparkBase.IdleMode;
// Hardware
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import frc.robot.utils.hardware.*;

// Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.NOMINAL_VOLTAGE;
import static frc.robot.constants.RobotConstants.ShooterConstants.*;

public class Shooter extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ShooterInputs {
        public boolean endLidarTripped;
        public boolean beginLidarTripped;
    }

    private Vortex shooterMotor1;
    private Vortex shooterMotor2;
    private Vortex feedMotor;
    private Lidar endLidar;
    private Lidar beginLidar;

    private ShooterInputsAutoLogged shooterInputs;

    public Shooter() {
        shooterMotor1 = VortexBuilder.create(SHOOTER_MOTOR_1_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withIdleMode(IdleMode.kBrake)
            .withCurrentLimit(80)
            .withInversion(true)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus2, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus5, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus6, 500)
            .build();

        shooterMotor2 = VortexBuilder.create(SHOOTER_MOTOR_2_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .withCurrentLimit(80)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus2, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus5, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus6, 500)
            .build();
        
        feedMotor = VortexBuilder.create(FEED_MOTOR_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withIdleMode(IdleMode.kBrake)
            .withCurrentLimit(40)
            .withInversion(true)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus2, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus5, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus6, 500)
            .build();

        try {
            Thread.sleep(300);
        } catch(InterruptedException e) {
            
        }

        endLidar = new Lidar(LIDAR_1_ID);
        beginLidar = new Lidar(LIDAR_2_ID);

        shooterInputs = new ShooterInputsAutoLogged();
    }

    public boolean endLidarTripped() {
        return endLidar.getRangeMeters() <= MAX_EMPTY_LIDAR_DISTANCE;
    }

    public boolean beginLidarTripped() {
        return beginLidar.getRangeMeters() <= MAX_EMPTY_LIDAR_DISTANCE;
    }

    public boolean lidarsTripped() {
        return endLidarTripped() && beginLidarTripped();
    }

    public boolean lidarTripped() {
        return endLidarTripped() || beginLidarTripped();
    }

    public void rev(double percent) {
        shooterMotor1.set(percent);
        shooterMotor2.set(percent);
    }

    public void runMotors(double power1, double power2) {
        shooterMotor1.set(power1);
        shooterMotor2.set(power2);
    }

    public void feed(double percent) {
        feedMotor.set(percent);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        shooterMotor1.log(subdirectory + "/" + humanReadableName, "ShooterMotor1");
        shooterMotor2.log(subdirectory + "/" + humanReadableName, "ShooterMotor2");
        feedMotor.log(subdirectory + "/" + humanReadableName, "FeedMotor");
        endLidar.log(subdirectory + "/" + humanReadableName, "EndFeedLidar");
        beginLidar.log(subdirectory + "/" + humanReadableName, "BeginFeedLidar");

        shooterInputs.endLidarTripped = endLidarTripped();
        shooterInputs.beginLidarTripped = beginLidarTripped();

        Logger.processInputs(subdirectory + "/" + humanReadableName, shooterInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Shooter");
    }
}
