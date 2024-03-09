package frc.robot.constants;

// Utilities
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.paths.TrajectoryContainer;

// Math
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
    private AutoConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static class PathFollowConstants {
        private PathFollowConstants() {
            throw new UnsupportedOperationException("Cannot construct a constants class");
        }

        public static final double POSITION_PID_P = 2.0;
        public static final double POSITION_PID_I = 0.0;
        public static final double POSITION_PID_D = 0.0;
        
        public static final double ROTATION_PID_P = 2.1;
        public static final double ROTATION_PID_I = 0.015;
        public static final double ROTATION_PID_D = 0.0;

        public static final PIDController X_CONTROLLER = new PIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D);
        public static final PIDController Y_CONTROLLER = new PIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D);
        public static final PIDController THETA_CONTROLLER = new PIDController(ROTATION_PID_P, ROTATION_PID_I, ROTATION_PID_D);

        public static final Pose2d DEFAULT_POSE_DELTA = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(20));

        static {
            THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        };
    }

    
    public static  class Paths {
        public static final SendableChooser<Supplier<Command>> AUTO_CHOOSER = new SendableChooser<>();
        public static final SendableChooser<String> CONTAINER_CHOOSER = new SendableChooser<>();

        private static TrajectoryContainer redTrajectories;
        private static TrajectoryContainer blueTrajectories;

        public static TrajectoryContainer getBlueTrajectories() {
            return blueTrajectories;
        }

        public static TrajectoryContainer getRedTrajectories() {
            return redTrajectories;
        }

        private static TrajectoryContainer globalTrajectories = null;

        public static void setGlobalTrajectories(TrajectoryContainer trajectories) {
            if(globalTrajectories != null) {
                throw new IllegalStateException("setGlobalTrajectories has already been set");
            }

            globalTrajectories = trajectories;
        }

        public static TrajectoryContainer getGlobalTrajectories() {
            return globalTrajectories;
        }

        public static boolean hasGlobalTrajectories() {
            return globalTrajectories != null;
        }

        public static void initializeTrajectories() {
            redTrajectories = new TrajectoryContainer("Red");
            blueTrajectories = new TrajectoryContainer("Blue");
        }
    }
}
