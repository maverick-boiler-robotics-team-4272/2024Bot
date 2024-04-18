package frc.robot.precomputations;

import static frc.robot.constants.UniversalConstants.FIELD_WIDTH_METERS;

import java.io.*;

import org.json.simple.*;
import org.json.simple.parser.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

class MirrorException extends RuntimeException {
    public MirrorException() {
        super();
    }

    public MirrorException(String message) {
        super(message);
    }

    public MirrorException(Throwable cause) {
        super(cause);
    }

    public MirrorException(String message, Throwable cause) {
        super(message, cause);
    }
}

public class Mirror {
    public static void main(String[] args) {
        // For mirroring a path:
        // mirrorPath([path name without side suffix], [side suffix]);
        mirrorPath("P6", "Blue");
        mirrorPath("P67", "Blue");
        mirrorPath("P65", "Blue");
        mirrorPath("P876", "Blue");
        mirrorPath("P456", "Blue");
        mirrorPath("P123Plus2", "Blue");
        mirrorPath("P1238", "Blue");
    }

    @SuppressWarnings({ "unchecked", "unused" })
    private static void mirrorPath(String pathName, String from) {
        String to = from == "Red" ? "Blue" : "Red";

        try(BufferedReader reader = new BufferedReader(new FileReader("./src/main/deploy/pathplanner/paths/" + from + " " + pathName + ".path"))) {
            JSONParser parser = new JSONParser();
            JSONObject obj = (JSONObject) parser.parse(reader);
            JSONArray waypoints = (JSONArray)obj.get("waypoints");

            for(int i = 0; i < waypoints.size(); i++) {
                JSONObject waypoint = (JSONObject) waypoints.get(i);

                JSONObject anchor = (JSONObject) waypoint.get("anchor");
                JSONObject prevControl = (JSONObject) waypoint.get("prevControl");
                JSONObject nextControl = (JSONObject) waypoint.get("nextControl");
                String linkedName = (String) waypoint.get("linkedName");

                if(linkedName != null) {
                    if(!linkedName.startsWith(from)) {
                        throw new MirrorException("Linked name " + linkedName + " must start with " + from);
                    } else {
                        linkedName = to + linkedName.substring(from.length());

                        waypoint.put("linkedName", linkedName);
                    }
                }

                if (anchor != null)
                    flipPoint(anchor);

                if (prevControl != null)
                    flipPoint(prevControl);

                if (nextControl != null)
                    flipPoint(nextControl);
            }

            JSONArray rotationTargets = (JSONArray) obj.get("rotationTargets");

            for(int i = 0; i < rotationTargets.size(); i++) {
                JSONObject target = (JSONObject)rotationTargets.get(i);

                flipRotation(target);
            }

            JSONObject endState = (JSONObject) obj.get("goalEndState");
            JSONObject previewState = (JSONObject) obj.get("previewStartingState");
            
            if (endState != null)
                flipRotation(endState);
            
            if (previewState != null)
                flipRotation(previewState);

            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            
            FileWriter writer = new FileWriter("./src/main/deploy/pathplanner/paths/" + to + " " + pathName + ".path");

            mapper.writeValue(writer, obj);

            writer.close();
        } catch(Exception e) {
            System.out.println("Error while mirroring path " + from + " " + pathName);
            e.printStackTrace();
        }
    }

    @SuppressWarnings({ "unchecked" })
    private static void flipPoint(JSONObject point) {
        double x = ensureDouble(point.get("x"));

        x = FIELD_WIDTH_METERS - x;

        point.put("x", x); // Suck it warning
    }

    @SuppressWarnings({ "unchecked" })
    private static void flipRotation(JSONObject rotation) {
        Double r = ensureDouble(rotation.get("rotation"));
        String name = "rotation";

        if(r == null) {
            r = ensureDouble(rotation.get(name = "rotationDegrees"));
        }

        if(r == null) {
            throw new MirrorException(new IllegalArgumentException("Passed in object must have either a rotation or rotationDegrees property"));
        }

        double rot = r;
        rot = 180 - rot;

        rotation.put(name, rot);
    }

    private static Double ensureDouble(Object obj) {
        if(obj == null) {
            return null;
        } if(obj instanceof Double) {
            return (Double)obj;
        } else if(obj instanceof Long) {
            return ((Long)obj).doubleValue();
        }
        
        throw new MirrorException(new IllegalArgumentException("Passed in object must be either a Long or a Double"));
    }
}
