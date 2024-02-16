package frc.robot.limelight;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.constants.UniversalConstants.*;

public final class Limelight {
    public enum LEDMode {
        kPipeline,
        kOff,
        kBlink,
        kOn
    }

    private final String tableName;

    // TODO: Tune this size
    private static final int FILTER_SIZE = 10;

    private MedianFilter xPositionFilter = new MedianFilter(FILTER_SIZE);
    private MedianFilter yPositionFilter = new MedianFilter(FILTER_SIZE);
    private MedianFilter thetaDegsFilter = new MedianFilter(FILTER_SIZE);
    
    private double filteredXPosition = 0.0;
    private double filteredYPosition = 0.0;
    private double filteredThetaDegs = 0.0;


    
    private static final Map<String, Limelight> limelightMap = new HashMap<>();

    private Limelight(String tableName) {
        if(tableName == "" || tableName == null) {
            tableName = "limelight";
        } else {
            tableName = "limelight-" + tableName;
        }

        this.tableName = tableName;
    }

    public double[] getBotPose() {
        double[] pose = LimelightHelpers.getBotPose(tableName);
        if(pose.length != 7) return new double[7];
        return pose;
    }

    public double[] getBotPoseInTargetSpace() {
        double[] pose = LimelightHelpers.getBotPose_TargetSpace(tableName);
        if(pose.length != 7) return new double[7];
        return pose;
    }

    public Pose2d getRobotPose() {        
        return new Pose2d(filteredXPosition + FIELD_HALF_WIDTH_METERS, filteredYPosition + FIELD_HALF_HEIGHT_METERS, Rotation2d.fromDegrees(filteredThetaDegs));
    }

    public Pose2d getUnfilteredPose() {
        double[] pose = getBotPose();

        return new Pose2d(pose[0] + FIELD_HALF_WIDTH_METERS, pose[1] + FIELD_HALF_HEIGHT_METERS, Rotation2d.fromDegrees(pose[5]));
    }

    public void filterPosition() {
        double[] pose = getBotPose();

        if(pose[0] == 0 && pose[1] == 0 && pose[5] == 0) {
            // Filler for invalid
            filteredXPosition = 0;
            filteredYPosition = 0;
            filteredThetaDegs = 0;
            return;
        }

        filteredXPosition = xPositionFilter.calculate(pose[0]);
        filteredYPosition = yPositionFilter.calculate(pose[1]);
        filteredThetaDegs = thetaDegsFilter.calculate(pose[5]);
    }

    public boolean isValidTarget() {
        double[] pose = getBotPose();

        return !(pose[0] == 0 && pose[1] == 0 && pose[5] == 0);
    }

    public double getTX() {
        return LimelightHelpers.getTX(tableName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(tableName);
    }

    public void setCropWindow(double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
        LimelightHelpers.setCropWindow(tableName, cropXMin, cropXMax, cropYMin, cropYMax);
    }

    public void setLEDMode(LEDMode mode) {
        if(mode == null) return;
        LimelightHelpers.setLimelightNTDouble(tableName, "ledMode", mode.ordinal());
    }

    public static Limelight getLimelight(String name) {
        if(!limelightMap.containsKey(name)) {
            limelightMap.put(name, new Limelight(name));
        }

        return limelightMap.get(name);
    }
}