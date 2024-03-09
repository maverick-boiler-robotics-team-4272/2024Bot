package frc.robot.utils.paths;

// Java Util
import java.util.*;

// Files
import java.io.*;
import edu.wpi.first.wpilibj.Filesystem;

// JSON
import org.json.simple.*;
import org.json.simple.parser.JSONParser;


// Pathplanner
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.auto.CommandUtil;

import edu.wpi.first.math.Pair;
// Math
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Commands
import edu.wpi.first.wpilibj2.command.Command;

public class TrajectoryContainer {
    public static class Path {
        public final PathPlannerTrajectory trajectory;
        public final Rotation2d initialPathRotation;
        public final PathPlannerPath path;
        public final List<Pair<Double, Command>> events;
        public final List<Pair<Double, Command>> pauses;

        public Path(String name, ChassisSpeeds initialSpeeds, Rotation2d initialRotation) {
            path = PathPlannerPath.fromPathFile(name);
            trajectory = path.getTrajectory(initialSpeeds, initialRotation);
            RotationTarget target = path.getPoint(0).rotationTarget;

            if(target != null) {
                initialPathRotation = target.getTarget();
            } else {
                throw new IllegalStateException(name + ": Path must have a rotation target at the beginning. This is the initial rotation of the robot on the path.");
            }

            events = new ArrayList<>();
            pauses = new ArrayList<>();
            
            try (BufferedReader br = new BufferedReader(new FileReader(Filesystem.getDeployDirectory() + "/pathplanner/paths/" + name + ".path"))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                for (var markerJson : (JSONArray) json.get("eventMarkers")) {
                    JSONObject marker = (JSONObject) markerJson;
                    String markerName = (String) marker.get("name");
                    Double waypointPos = (Double) marker.get("waypointRelativePos");

                    int index = (int) Math.round(waypointPos / PathSegment.RESOLUTION);
                    double time = trajectory.getState(index).timeSeconds;

                    Command command = CommandUtil.commandFromJson((JSONObject) marker.get("command"), false);
                    
                    if(markerName.equals("Pause")) {
                        pauses.add(Pair.of(time, command));
                    } else {
                        events.add(Pair.of(time, command));
                    }
                }
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            
        }

        public Path(String name) {
            this(name, INITIAL_SPEEDS, INITIAL_ROTATION);
        }
    }

    private static final ChassisSpeeds INITIAL_SPEEDS = new ChassisSpeeds(0, 0, 0);
    private static final Rotation2d INITIAL_ROTATION = new Rotation2d(180);

    public final Path TWO_CENTER_RUSH;
    public final Path THREE_PIECE_CLOSE;
    public final Path TWO_STAGE_RUSH;
    public final Path P_14;
    
    public final Path P_1238;

    public TrajectoryContainer(String prefix) {
        TWO_CENTER_RUSH = new Path(prefix + " Two Center Rush");
        THREE_PIECE_CLOSE = new Path(prefix + " Three Piece Close");
        TWO_STAGE_RUSH = new Path(prefix + " Two Stage Rush");

        //New Naming convention
        //P for piece ### labling the pieces and the order they are grabed
        P_14 = new Path(prefix + " P14");
        P_1238 = new Path(prefix + " P1238");
    }
}
