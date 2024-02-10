package frc.robot.utils;

import static frc.robot.constants.UniversalConstants.*;

import java.util.*;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class AStarPathfinding implements Pathfinder {
    private static class GridPoint {
        public int x;
        public int y;

        public GridPoint(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public boolean equals(Object obj) {
            if(obj instanceof GridPoint) {
                GridPoint g = (GridPoint)obj;
                return this.x == g.x && this.y == g.y;
            }

            return false;
        }

        @Override
        public String toString() {
            return "GridPoint(x = %d, y = %d)".formatted(x, y);
        }

        @Override
        public int hashCode() {
            return Objects.hash(this.x, this.y);
        }
    }

    private static class TranslationList {
        public static class TranslationNode {
            public Translation2d position;
            public TranslationNode prev;
            public TranslationNode next;

            public TranslationNode(Translation2d position) {
                this.position = position;

                this.prev = null;
                this.next = null;
            }
        }

        public TranslationNode head;
        public TranslationNode tail;
        public int size = 0;

        public TranslationList() {
            this.head = null;
            this.tail = null;
        }

        public void append(Translation2d point) {
            if(this.head == null) {
                this.head = new TranslationNode(point);
                this.tail = this.head;
            } else {
                TranslationNode newNode = new TranslationNode(point);

                newNode.prev = this.tail;
                this.tail.next = newNode;
                this.tail = newNode;
            }

            size++;
        }
    }

    private double resolution = 0.2;
    private double controlProportion = 0.33;
    private double anchorProportion = 0.8;

    private double nodeSize;
    private boolean[][] navgrid;
    private int gridWidth = 0;
    private int gridHeight = 0;

    private GridPoint startPoint = new GridPoint(0, 0);
    private GridPoint endPoint = new GridPoint(0, 0);

    private Translation2d goalPosition;
    private Translation2d startPosition;
    
    private Map<GridPoint, GridPoint> cameFromMap = new HashMap<>();
    private Map<GridPoint, Double> gScores = new HashMap<>();
    private Map<GridPoint, Double> fScores = new HashMap<>();
    private Set<GridPoint> openSet = new HashSet<GridPoint>();

    public AStarPathfinding(double nodeSize) {
        gridWidth = (int)(Math.ceil(FIELD_HEIGHT_METERS / nodeSize));
        gridHeight = (int)(Math.ceil(FIELD_HEIGHT_METERS / nodeSize));

        navgrid = new boolean[gridWidth][gridHeight];

        this.nodeSize = nodeSize;
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        this.startPosition = goalPosition;
        
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        this.goalPosition = startPosition;
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        for(int i = 0; i < gridWidth; i++) {
            for(int j = 0; j < gridWidth; j++) {
                navgrid[i][j] = false;
            }
        }

        for(Pair<Translation2d, Translation2d> bounds : obs) {
            Translation2d first = bounds.getFirst();
            Translation2d second = bounds.getSecond();
            int minX = (int)Math.floor(Math.min(first.getX(), second.getX()) / nodeSize);
            int minY = (int)Math.floor(Math.min(first.getY(), second.getY()) / nodeSize);
            int maxX = (int)Math.ceil(Math.max(first.getX(), second.getX()) / nodeSize);
            int maxY = (int)Math.ceil(Math.max(first.getY(), second.getY()) / nodeSize);

            minX = Math.max(minX, 0);
            minY = Math.max(minY, 0);
            maxX = Math.min(maxX, gridWidth - 1);
            maxY = Math.min(maxY, gridHeight - 1);

            for(int i = minX; i < maxX; i++) {
                for(int j = minY; j < maxY; j++) {
                    navgrid[i][j] = true;
                }
            }
        }
        
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        if(!generatePath())
            return null;
        
        List<PathPoint> pathPoints = generatePathPoints();

        return PathPlannerPath.fromPathPoints(pathPoints, constraints, goalEndState);
    }

    @Override
    public boolean isNewPathAvailable() {
        return false;
    }

    private double dist(GridPoint a, GridPoint b) {
        return navgrid[a.x][a.y] || navgrid[b.x][b.y] ? Double.POSITIVE_INFINITY : Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
        // return navgrid[a.x][a.y] || navgrid[b.x][b.y] ? Double.POSITIVE_INFINITY : Math.hypot(a.x - b.x, a.y - b.y);
    }

    private double heuristic(GridPoint g) {
        // return Math.abs(endPoint.x - g.x) + Math.abs(endPoint.y - g.y);
        return Math.hypot(endPoint.x - g.x, endPoint.y - g.y);
    }

    private boolean generatePath() {
        if(startPosition.getX() > goalPosition.getX()) {
            startPoint.x = (int)(Math.ceil(startPosition.getX() / nodeSize));
            endPoint.x = (int)(Math.floor(goalPosition.getX() / nodeSize));
        } else {
            startPoint.x = (int)(Math.floor(startPosition.getX() / nodeSize));
            endPoint.x = (int)(Math.ceil(goalPosition.getX() / nodeSize));
        }

        if(startPosition.getY() > goalPosition.getY()) {
            startPoint.y = (int)(Math.ceil(startPosition.getY() / nodeSize));
            endPoint.y = (int)(Math.floor(goalPosition.getY() / nodeSize));
        } else {
            startPoint.y = (int)(Math.floor(startPosition.getY() / nodeSize));
            endPoint.y = (int)(Math.ceil(goalPosition.getY() / nodeSize));
        }

        cameFromMap.clear();
        openSet.clear();
        fScores.clear();
        gScores.clear();


        fScores.put(startPoint, heuristic(startPoint));
        gScores.put(startPoint, 0.0);
        
        openSet.add(startPoint);

        while(!openSet.isEmpty()) {
            GridPoint current = Collections.min(openSet, (a, b) -> {
                return Double.compare(fScores.get(a), fScores.get(b));
            });

            if(current.equals(endPoint)) {
                return true;
            }

            openSet.remove(current);

            for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++) {
                    if(i == 1 && j == 1)
                        continue;
                    int xInd = current.x + i - 1;
                    int yInd = current.y + j - 1;
                    if(xInd < 0 || xInd >= gridWidth || yInd < 0 || yInd >= gridHeight)
                        continue;
                    
                    GridPoint neighbor = new GridPoint(xInd, yInd);
                    double tentativeGScore = gScores.get(current) + dist(current, neighbor);
                    if(tentativeGScore < gScores.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                        if(openSet.contains(neighbor)) {
                            for(GridPoint p : openSet) {
                                if(p.equals(neighbor)) {
                                    neighbor = p;
                                    break;
                                }
                            }
                        } else {
                            openSet.add(neighbor);
                        }

                        cameFromMap.put(neighbor, current);

                        gScores.put(neighbor, tentativeGScore);
                        fScores.put(neighbor, tentativeGScore + heuristic(neighbor));
                    }
                }
            }
        }

        return false;
    }

    private List<PathPoint> generatePathPoints() {
        List<PathPoint> pathPoints = new ArrayList<>();

        TranslationList pathPositions = new TranslationList();

        GridPoint currentPoint = endPoint;

        while(cameFromMap.containsKey(currentPoint)) {
            pathPositions.append(new Translation2d(currentPoint.x * nodeSize, currentPoint.y * nodeSize));

            currentPoint = cameFromMap.get(currentPoint);
        }

        int size = pathPositions.size;

        TranslationList bezierPositions = new TranslationList();

        bezierPositions.append(pathPositions.head.position);

        bezierPositions.append(
            pathPositions.head.next.position
                .minus(pathPositions.head.position)
                .times(controlProportion)
                .plus(pathPositions.head.position)
        );

        TranslationList.TranslationNode currentNode = pathPositions.head.next;

        for(int i = 1; i < size - 1; i++) {
            Translation2d last = currentNode.prev.position;
            Translation2d current = currentNode.position;
            Translation2d next = currentNode.next.position;

            Translation2d anchor1 = current.minus(last).times(anchorProportion).plus(last);
            Translation2d anchor2 = current.minus(next).times(anchorProportion).plus(next);

            double controlDist = anchor1.getDistance(anchor2) * controlProportion;

            Translation2d prevControl1 = last.minus(anchor1).times(controlProportion).plus(anchor1);
            Translation2d nextControl1 =
                new Translation2d(controlDist, anchor1.minus(prevControl1).getAngle()).plus(anchor1);

            Translation2d prevControl2 =
                new Translation2d(controlDist, anchor2.minus(next).getAngle()).plus(anchor2);
            Translation2d nextControl2 = next.minus(anchor2).times(controlProportion).plus(anchor2);

            bezierPositions.append(prevControl1);
            bezierPositions.append(anchor1);
            bezierPositions.append(nextControl1);

            bezierPositions.append(prevControl2);
            bezierPositions.append(anchor2);
            bezierPositions.append(nextControl2);

            currentNode = currentNode.next;
        }

        bezierPositions.append(
            pathPositions.tail.prev.position
                .minus(pathPositions.tail.position)
                .times(controlProportion)
                .plus(pathPositions.tail.position)
        );

        bezierPositions.append(pathPositions.tail.position);

        currentNode = bezierPositions.head;

        for(int i = 0; i < bezierPositions.size - 1; i += 3) {
            Translation2d start = currentNode.position;
            currentNode = currentNode.next;

            Translation2d ctpt1 = currentNode.position;
            currentNode = currentNode.next;

            Translation2d ctpt2 = currentNode.position;
            currentNode = currentNode.next;

            Translation2d end = currentNode.position;

            for(double t = 0.0; t <= 1.0; t += resolution) {
                pathPoints.add(
                    new PathPoint(
                        GeometryUtil.cubicLerp(start, ctpt1, ctpt2, end, t)
                    )
                );
            }
        }

        return pathPoints;
    }
}
