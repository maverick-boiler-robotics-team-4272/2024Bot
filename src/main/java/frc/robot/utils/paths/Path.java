package frc.robot.utils.paths;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathSegment;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Path {
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
            System.out.println(name + " og Rotation: " + initialRotation);
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

    public Path(PathPlannerPath path, ChassisSpeeds initialSpeeds, Rotation2d initialRotation) {
        this.path = path;
        this.trajectory = this.path.getTrajectory(initialSpeeds, initialRotation);
        this.initialPathRotation = initialRotation;

        var markers = path.getEventMarkers();

        this.events = new ArrayList<>();

        for(var marker : markers) {
            int index = (int) Math.round(marker.getWaypointRelativePos() / PathSegment.RESOLUTION);
            double time = this.trajectory.getState(index).timeSeconds;

            events.add(new Pair<Double,Command>(time, marker.getCommand()));
        }

        this.pauses = List.of();
    }

    public Path(PathPlannerPath path) {
        this(path, DEFAULT_INITIAL_SPEEDS, DEFAULT_INITIAL_ROTATION);
    }

    public Path(String name) {
        this(name, DEFAULT_INITIAL_SPEEDS, DEFAULT_INITIAL_ROTATION);
    }

    public static final ChassisSpeeds DEFAULT_INITIAL_SPEEDS = new ChassisSpeeds(0, 0, 0);
    public static final Rotation2d DEFAULT_INITIAL_ROTATION = Rotation2d.fromDegrees(180);
}
