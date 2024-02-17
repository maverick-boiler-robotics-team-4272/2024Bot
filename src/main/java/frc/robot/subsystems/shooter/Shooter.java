package frc.robot.subsystems.shooter;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.RobotConstants.ShooterConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.*;
public class Shooter extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ShooterInputs {

    }

    private Vortex shooterMotor1;
    private Vortex shooterMotor2;
    private Vortex feedMotor;
    private Lidar lidar;

    private ShooterInputsAutoLogged shooterInputs;

    public Shooter() {
        shooterMotor1 = VortexBuilder.createWithDefaults(SHOOTER_MOTOR_1_ID)
            .build();
        shooterMotor2 = VortexBuilder.createWithDefaults(SHOOTER_MOTOR_2_ID)
            .asFollower(shooterMotor1, true)
            .build();
        feedMotor = VortexBuilder.createWithDefaults(FEED_MOTOR_ID)
            .build();

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

        Logger.processInputs(subdirectory + "/" + humanReadableName, shooterInputs);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Shooter");
    }
}
