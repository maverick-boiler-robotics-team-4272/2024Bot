package frc.robot.precomputations;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class BoundingBox {
    private static class Bounds {
        public int x, y, width, height;

        public Bounds(int x, int y, int width, int height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }

        public boolean pointInBox(int x, int y) {
            return x >= this.x && x <= this.x + width && y >= this.y && y <= this.y + height;
        }

        @Override
        public String toString() {
            return String.format("Bounding(x = %d, y = %d, width = %d, height = %d)", y + 1, x + 1, height, width);
        }
    }

    public static void main(String[] args) throws Exception {
        StringBuilder builder = new StringBuilder();
        try(BufferedReader reader = new BufferedReader(new FileReader("./src/main/deploy/pathplanner/navgrid.json"))) {
            builder.append(reader.readLine());
        } catch(Exception e) {
            System.out.println("An error occurred reading the file");
            throw new RuntimeException(e);
        }

        JSONObject obj = (JSONObject) new JSONParser().parse(builder.toString());
        double nodeSize = (Double) obj.get("nodeSizeMeters");

        JSONArray grid = (JSONArray) obj.get("grid");
        
        boolean[][] navgrid = new boolean[grid.size()][((JSONArray) grid.get(0)).size()];

        for(int i = 0; i < grid.size(); i++) {
            JSONArray gridArr = (JSONArray) grid.get(i);

            for(int j = 0; j < gridArr.size(); j++) {
                navgrid[i][j] = (Boolean) gridArr.get(j);
            }
        }

        List<Bounds> bbs = new ArrayList<>();
        
        for(int i = 0; i < navgrid.length; i++) {
            for(int j = 0; j < navgrid[i].length; j++) {
                if(isPointInAnyBound(bbs, i, j) || !navgrid[i][j])
                    continue;
                floodFill(navgrid, bbs, i, j);
            }
        }

        System.out.println("[");
        bbs.forEach(b -> {
            System.out.print(b);
            System.out.println(",");
        });
        System.out.println("]");

        FileWriter writer = new FileWriter("./src/main/deploy/precompout/bounds.txt");

        writer.write("List.of(\n");
        bbs.forEach(b -> {
            try {
                writer.write("new Pair<Translation2d, Translation2d>(new Translation2d(%.2f, %.2f), new Translation2d(%.2f, %.2f)),\n".formatted(b.y * nodeSize, b.x * nodeSize, (b.y + b.height) * nodeSize, (b.x + b.width) * nodeSize));
            } catch(Exception e) {
                System.out.println("I don't care, but error occurred");
            }
        });
        writer.write(')');
        
        writer.close();

        for(int i = 0; i < navgrid.length; i++) {
            for(int j = 0; j < navgrid[i].length; j++) {
                if(navgrid[i][j]) {
                    if(!isPointInAnyBound(bbs, i, j)) {
                        System.out.println("Cell not in grid: %d, %d".formatted(j, i));
                    }
                }
            }
        }
    }

    private static boolean isPointInAnyBound(List<Bounds> bounds, int x, int y) {
        for(Bounds b : bounds) {
            if(b.pointInBox(x, y))
                return true;
        }

        return false;
    }

    private static void floodFill(boolean[][] navgrid, List<Bounds> bounds, int x, int y) {
        int width = 0;
        for(int i = x; i < navgrid.length; i++) {
            if(isPointInAnyBound(bounds, i, y) || !navgrid[i][y]) {
                break;
            }

            width++;
        }

        int height = 0;
        for(int j = y; j < navgrid[x].length; j++) {
            boolean stop = false;
            for(int i = x; i < x + width; i++) {
                if(isPointInAnyBound(bounds, i, j) || !navgrid[i][j]) {
                    stop = true;
                    break;
                }
            }

            if(stop)
                break;
            
            height++;
        }

        bounds.add(new Bounds(x, y, width, height));
    }
}
