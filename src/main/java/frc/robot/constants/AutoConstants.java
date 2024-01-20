package frc.robot.constants;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.TrajectoryContainer;

public class AutoConstants {
    private AutoConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static class PathFollowConstants {
        private PathFollowConstants() {
            throw new UnsupportedOperationException("Cannot construct a constants class");
        }

        public static final double POSITION_PID_P = 1.0;
        public static final double POSITION_PID_I = 0.0;
        public static final double POSITION_PID_D = 0.0;
        
        public static final double ROTATION_PID_P = 1.0;
        public static final double ROTATION_PID_I = 0.0;
        public static final double ROTATION_PID_D = 0.0;

        public static final PIDController X_CONTROLLER = new PIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D);
        public static final PIDController Y_CONTROLLER = new PIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D);
        public static final PIDController THETA_CONTROLLER = new PIDController(ROTATION_PID_P, ROTATION_PID_I, ROTATION_PID_D);

        public static final Pose2d DEFAULT_POSE_DELTA = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2));

        static {
            THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        };
    }
}
