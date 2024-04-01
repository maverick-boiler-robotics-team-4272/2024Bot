package frc.robot.precomputations;

import static frc.robot.constants.UniversalConstants.FIELD_HALF_HEIGHT_METERS;
import static frc.robot.constants.UniversalConstants.FIELD_HALF_WIDTH_METERS;
import static frc.robot.constants.UniversalConstants.FIELD_HEIGHT_METERS;
import static frc.robot.constants.UniversalConstants.FIELD_WIDTH_METERS;

import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectWriter;
import com.fasterxml.jackson.databind.SerializationFeature;

public class NavGridCreation {
    private static class Vec2 implements Cloneable {
        public double x;
        public double y;

        public Vec2(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Vec2 add(Vec2 v) {
            this.x += v.x;
            this.y += v.y;

            return this;
        }

        public Vec2 sub(Vec2 v) {
            this.x -= v.x;
            this.y -= v.y;

            return this;
        }

        public Vec2 invert() {
            this.x *= -1;
            this.y *= -1;

            return this;
        }

        public Vec2 multiply(double s) {
            this.x *= s;
            this.y *= s;

            return this;
        }



        public Vec2 clone() {
            Vec2 clone = new Vec2(0, 0);

            clone.x = this.x;
            clone.y = this.y;

            return clone;
        }

        public Vec2 copy(Vec2 v) {
            this.x = v.x;
            this.y = v.y;

            return this;
        }

        public double length() {
            return Math.hypot(x, y);
        }

        public Vec2 normalize() {
            double l = length();

            if(l == 0) {
                this.x = this.y = 0;
            } else {
                l = 1 / l;
                this.x *= l;
                this.y *= l;
            }

            return this;
        }

        public double cross(Vec2 a) {
            return this.x * a.y - this.y * a.x;
        }

        public double dot(Vec2 a) {
            return this.x * a.x + this.y * a.y;
        }
    }
        
    private static final double NODE_SIZE = 0.1;

    private static final Vec2 LEFT_TRUSS_BLUE = new Vec2(5.61, 5.37);
    private static final Vec2 CENTER_TRUSS_BLUE = new Vec2(3.39, 4.10);
    private static final Vec2 RIGHT_TRUSS_BLUE = new Vec2(5.61, 2.85);

    private static final Vec2 LEFT_TRUSS_RED = new Vec2(10.96, 5.34);
    private static final Vec2 CENTER_TRUSS_RED = new Vec2(13.20, 4.10);
    private static final Vec2 RIGHT_TRUSS_RED = new Vec2(10.96, 2.82);

    private static final Vec2 LOWER_LEFT_SUB_BLUE = new Vec2(0.00, 6.49);
    private static final Vec2 UPPER_LEFT_SUB_BLUE = new Vec2(0.90, 6.04);
    private static final Vec2 LOWER_RIGHT_SUB_BLUE = new Vec2(0.00, 4.59);
    private static final Vec2 UPPER_RIGHT_SUB_BLUE = new Vec2(0.90, 5.00);

    private static final Vec2 LOWER_LEFT_SUB_RED = new Vec2(15.66, 6.49);
    private static final Vec2 UPPER_LEFT_SUB_RED = new Vec2(15.66, 6.04);
    private static final Vec2 LOWER_RIGHT_SUB_RED = new Vec2(16.49, 4.59);
    private static final Vec2 UPPER_RIGHT_SUB_RED = new Vec2(15.66, 5.00);

    
    private static final Vec2 BLUE_LEFT_STAGE_LINE = LEFT_TRUSS_BLUE.clone().sub(CENTER_TRUSS_BLUE);
    private static final Vec2 BLUE_LEFT_STAGE_DIR = BLUE_LEFT_STAGE_LINE.clone().normalize();
    private static final Vec2 BLUE_BACK_STAGE_LINE = RIGHT_TRUSS_BLUE.clone().sub(LEFT_TRUSS_BLUE);
    private static final Vec2 BLUE_BACK_STAGE_DIR = BLUE_BACK_STAGE_LINE.clone().normalize();
    private static final Vec2 BLUE_RIGHT_STAGE_LINE = CENTER_TRUSS_BLUE.clone().sub(RIGHT_TRUSS_BLUE);
    private static final Vec2 BLUE_RIGHT_STAGE_DIR = BLUE_RIGHT_STAGE_LINE.clone().normalize();

    private static final Vec2 RED_LEFT_STAGE_LINE = LEFT_TRUSS_RED.clone().sub(CENTER_TRUSS_RED);
    private static final Vec2 RED_LEFT_STAGE_DIR = RED_LEFT_STAGE_LINE.clone().normalize();
    private static final Vec2 RED_BACK_STAGE_LINE = RIGHT_TRUSS_RED.clone().sub(LEFT_TRUSS_RED);
    private static final Vec2 RED_BACK_STAGE_DIR = RED_BACK_STAGE_LINE.clone().normalize();
    private static final Vec2 RED_RIGHT_STAGE_LINE = CENTER_TRUSS_RED.clone().sub(RIGHT_TRUSS_RED);
    private static final Vec2 RED_RIGHT_STAGE_DIR = RED_RIGHT_STAGE_LINE.clone().normalize();

    @SuppressWarnings({ "unchecked" })
    public static void main(String[] args) throws IOException {
        boolean[][] grid = new boolean[(int)Math.ceil(FIELD_WIDTH_METERS / NODE_SIZE)][(int)Math.ceil(FIELD_HEIGHT_METERS / NODE_SIZE)];

        final double STAGE_DISTANCE = 0.5;
        final double STAGE_DISTANCE_SQ = STAGE_DISTANCE * STAGE_DISTANCE;
        final double SUB_DISTANCE = 0.5;
        final double SUB_DISTANCE_SQ =  SUB_DISTANCE * SUB_DISTANCE;

        final boolean SUB_UNDER = false;

        Vec2 help = new Vec2(0, 0);

        // Update for stage
        for(int i = 0; i < grid.length; i++) {
            for(int j = 0; j < grid[i].length; j++) {
                Vec2 pos = new Vec2((i + 0.5) * NODE_SIZE, (j + 0.5) * NODE_SIZE);

                double blueCenterTrussDist = help.copy(pos).sub(CENTER_TRUSS_BLUE).dot(help);
                double blueLeftTrussDist = help.copy(pos).sub(LEFT_TRUSS_BLUE).dot(help);
                double blueRightTrussDist = help.copy(pos).sub(RIGHT_TRUSS_BLUE).dot(help);

                double redCenterTrussDist = help.copy(pos).sub(CENTER_TRUSS_RED).dot(help);
                double redLeftTrussDist = help.copy(pos).sub(LEFT_TRUSS_RED).dot(help);
                double redRightTrussDist = help.copy(pos).sub(RIGHT_TRUSS_RED).dot(help);

                double blueLeftDistAway = help.copy(pos).sub(CENTER_TRUSS_BLUE).cross(BLUE_LEFT_STAGE_DIR);
                double blueLeftDistAlong = help.dot(BLUE_LEFT_STAGE_DIR);

                double blueCenterDistAway = help.copy(pos).sub(LEFT_TRUSS_BLUE).cross(BLUE_BACK_STAGE_DIR);
                double blueCenterDistAlong = help.dot(BLUE_BACK_STAGE_DIR);

                double blueRightDistAway = help.copy(pos).sub(RIGHT_TRUSS_BLUE).cross(BLUE_RIGHT_STAGE_DIR);
                double blueRightDistAlong = help.dot(BLUE_RIGHT_STAGE_DIR);

                double redLeftDistAway = help.copy(pos).sub(CENTER_TRUSS_RED).cross(RED_LEFT_STAGE_DIR);
                double redLeftDistAlong = help.dot(RED_LEFT_STAGE_DIR);

                double redCenterDistAway = help.copy(pos).sub(LEFT_TRUSS_RED).cross(RED_BACK_STAGE_DIR);
                double redCenterDistAlong = help.dot(RED_BACK_STAGE_DIR);

                double redRightDistAway = help.copy(pos).sub(RIGHT_TRUSS_RED).cross(RED_RIGHT_STAGE_DIR);
                double redRightDistAlong = help.dot(RED_RIGHT_STAGE_DIR);

                double minDistAway;
                double minDistAlong;
                double maxDistAlong;

                if(
                    blueCenterTrussDist <= STAGE_DISTANCE_SQ ||
                    blueLeftTrussDist <= STAGE_DISTANCE_SQ ||
                    blueRightTrussDist <= STAGE_DISTANCE_SQ ||
                    redCenterTrussDist <= STAGE_DISTANCE_SQ ||
                    redLeftTrussDist <= STAGE_DISTANCE_SQ ||
                    redRightTrussDist <= STAGE_DISTANCE_SQ
                ) {
                    grid[i][j] = true;
                }

                if(Math.abs(blueLeftDistAway) < Math.abs(blueRightDistAway) && Math.abs(blueLeftDistAway) < Math.abs(blueCenterDistAway)) {
                    minDistAway = blueLeftDistAway;
                    minDistAlong = blueLeftDistAlong;
                    maxDistAlong = BLUE_LEFT_STAGE_LINE.length();
                } else if(Math.abs(blueRightDistAway) < Math.abs(blueLeftDistAway) && Math.abs(blueRightDistAway) < Math.abs(blueCenterDistAway)) {
                    minDistAway = blueRightDistAway;
                    minDistAlong = blueRightDistAlong;
                    maxDistAlong = BLUE_RIGHT_STAGE_LINE.length();
                } else {
                    minDistAway = blueCenterDistAway;
                    minDistAlong = blueCenterDistAlong;
                    maxDistAlong = BLUE_BACK_STAGE_LINE.length();
                }
                
                if(
                    (minDistAway >= -STAGE_DISTANCE && minDistAlong >= 0.0 && minDistAlong <= maxDistAlong && SUB_UNDER) // ||
                    // (RED_LEFT_STAGE_LINE.cross(help.copy(pos).sub(LEFT_TRUSS_RED)) < 0.0 && 
                    // RED_RIGHT_STAGE_LINE.cross(help.copy(pos).sub(RIGHT_TRUSS_RED)) < 0.0 &&
                    // RED_BACK_STAGE_LINE.cross(help.copy(pos).sub(CENTER_TRUSS_RED)) < 0.0)
                ) {
                    grid[i][j] = true;
                }

                if(Math.abs(redLeftDistAway) < Math.abs(redRightDistAway) && Math.abs(redLeftDistAway) < Math.abs(redCenterDistAway)) {
                    minDistAway = redLeftDistAway;
                    minDistAlong = redLeftDistAlong;
                    maxDistAlong = RED_LEFT_STAGE_LINE.length();
                } else if(Math.abs(redRightDistAway) < Math.abs(redLeftDistAway) && Math.abs(redRightDistAway) < Math.abs(redCenterDistAway)) {
                    minDistAway = redRightDistAway;
                    minDistAlong = redRightDistAlong;
                    maxDistAlong = RED_RIGHT_STAGE_LINE.length();
                } else {
                    minDistAway = redCenterDistAway;
                    minDistAlong = redCenterDistAlong;
                    maxDistAlong = RED_BACK_STAGE_LINE.length();
                }

                if(
                    (minDistAway <= STAGE_DISTANCE && minDistAlong >= 0.0 && minDistAlong <= maxDistAlong && SUB_UNDER)
                ) {
                    grid[i][j] = true;
                }
            }
        }

        // write json
        JSONObject writeObj = new JSONObject();
        JSONObject fieldSize = new JSONObject();

        JSONArray fieldGrid = new JSONArray();

        writeObj.put("field_size", fieldSize);

        fieldSize.put("x", FIELD_WIDTH_METERS);
        fieldSize.put("y", FIELD_HEIGHT_METERS);

        writeObj.put("nodeSizeMeters", NODE_SIZE);

        writeObj.put("grid", fieldGrid);

        for(int j = 0; j < grid[0].length; j++) {
            JSONArray row = new JSONArray();

            for(int i = 0; i < grid.length; i++) {
                row.add(grid[i][j]);
            }

            fieldGrid.add(row);
        }

        ObjectMapper mapper = new ObjectMapper();
        mapper.enable(SerializationFeature.INDENT_OUTPUT);
            
        FileWriter writer = new FileWriter("./src/main/deploy/pathplanner/navgrid.json");

        mapper.writeValue(writer, writeObj);
    }
}
