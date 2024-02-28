package frc.robot.precomputations;

import static frc.robot.constants.UniversalConstants.FIELD_WIDTH_METERS;

import java.io.*;

import org.json.simple.*;
import org.json.simple.parser.*;

public class Mirror {
    public static void main(String[] args) {
        mirrorPath("Two Center Rush", "Blue");
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
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    private static void flipPoint(JSONObject point) {
        double x = (Double) point.get("x");

        x = FIELD_WIDTH_METERS - x;

        point.put("x", x); // Suck it warning
    }
}
