package frc.robot.precomputations;

import static frc.robot.constants.UniversalConstants.FIELD_WIDTH_METERS;

import java.io.*;

import org.json.simple.*;
import org.json.simple.parser.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

public class Mirror {
    public static void main(String[] args) {
        mirrorPath("Three Piece Close", "Blue");
        mirrorPath("Two Center Rush", "Blue");
        mirrorPath("Two Stage Rush", "Blue");
    }

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

            flipRotation(endState);
            flipRotation(previewState);


            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            
            FileWriter writer = new FileWriter("./src/main/deploy/pathplanner/paths/" + to + " " + pathName + ".path");

            mapper.writeValue(writer, obj);

            writer.close();
        } catch(Exception e) {
            e.printStackTrace();
            System.out.println("Error in path " + pathName);
        }
    }

    private static void flipPoint(JSONObject point) {
        double x = ensureDouble(point.get("x"));

        x = FIELD_WIDTH_METERS - x;

        point.put("x", x); // Suck it warning
    }

    private static void flipRotation(JSONObject rotation) {
        Double r = ensureDouble(rotation.get("rotation"));
        String name = "rotation";

        if(r == null) {
            r = ensureDouble(rotation.get(name = "rotationDegrees"));
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
        
        throw new IllegalArgumentException("Passed in object must be either a Long or a Double");
    }
}
