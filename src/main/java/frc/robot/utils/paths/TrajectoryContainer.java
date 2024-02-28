package frc.robot.utils.paths;

// Pathplanner
import com.pathplanner.lib.path.*;

// Math
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TrajectoryContainer {
    public static class Path {
        public final PathPlannerTrajectory trajectory;
        public final PathPlannerPath path;

        public Path(String name, ChassisSpeeds initialSpeeds, Rotation2d initialRotation) {
            path = PathPlannerPath.fromPathFile(name);
            trajectory = path.getTrajectory(initialSpeeds, initialRotation);
        }

        public Path(String name) {
            this(name, INITIAL_SPEEDS, INITIAL_ROTATION);
        }
    }

    private static final ChassisSpeeds INITIAL_SPEEDS = new ChassisSpeeds(0, 0, 0);
    private static final Rotation2d INITIAL_ROTATION = new Rotation2d(180);

    public final Path TEST_PATH;

    public final Path TUNE_PATH;
    public final Path STRESS_TEST_PATH;

    public final Path TWO_CENTER_RUSH;
    public final Path THREE_PIECE_CLOSE;
    public final Path TWO_STAGE_RUSH;

    public TrajectoryContainer(String prefix) {
        TEST_PATH = new Path(prefix + " Test Path");
        TUNE_PATH = new Path(prefix + " Tune Path");
        STRESS_TEST_PATH = new Path(prefix + " Stress Test Path");

        TWO_CENTER_RUSH = new Path(prefix + " Two Center Rush");
        THREE_PIECE_CLOSE = new Path(prefix + " Three Piece Close");
        TWO_STAGE_RUSH = new Path(prefix + " Two Stage Rush");
    }
}
