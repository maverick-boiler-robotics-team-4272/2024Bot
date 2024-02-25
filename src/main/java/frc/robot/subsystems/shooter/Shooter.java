package frc.robot.subsystems.shooter;

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
import static frc.robot.constants.RobotConstants.ShooterConstants.*;

public class Shooter extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ShooterInputs {
        public boolean lidarTripped;
    }

    private Vortex shooterMotor1;
    private Vortex shooterMotor2;
    private Vortex feedMotor;
    private Lidar lidar;

    private ShooterInputsAutoLogged shooterInputs;

    public Shooter() {
        shooterMotor1 = VortexBuilder.createWithDefaults(SHOOTER_MOTOR_1_ID)
            .withCurrentLimit(80)
            // .withIdleMode(IdleMode.kCoast)
            .withInversion(true)
            .build();
        shooterMotor2 = VortexBuilder.createWithDefaults(SHOOTER_MOTOR_2_ID)
            .asFollower(shooterMotor1, true)
            .withCurrentLimit(80)
            // .withIdleMode(IdleMode.kCoast)
            .build();
        feedMotor = VortexBuilder.createWithDefaults(FEED_MOTOR_ID)
            .withCurrentLimit(40)
            .withInversion(true)
            .withIdleMode(IdleMode.kCoast)
            .build();

        
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);



        lidar = new Lidar(LIDAR_1_ID);

        shooterInputs = new ShooterInputsAutoLogged();
    }

    public boolean lidarTripped() {
        return lidar.getRangeMeters() <= MAX_EMPTY_LIDAR_DISTANCE;
    }

    public void rev(double percent) {
        shooterMotor1.set(percent);
    }

    public void feed(double percent) {
        feedMotor.set(percent);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        shooterMotor1.log(subdirectory + "/" + humanReadableName, "ShooterMotor1");
        shooterMotor2.log(subdirectory + "/" + humanReadableName, "ShooterMotor2");
        feedMotor.log(subdirectory + "/" + humanReadableName, "FeedMotor");
        lidar.log(subdirectory + "/" + humanReadableName, "FeedLidar");

        shooterInputs.lidarTripped = lidarTripped();

        Logger.processInputs(subdirectory + "/" + humanReadableName, shooterInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Shooter");
    }
}
