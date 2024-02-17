package frc.robot.limelight;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.Loggable;
import frc.robot.utils.PeriodicsUtil;
import frc.robot.utils.PeriodicsUtil.Periodic;

import static frc.robot.constants.UniversalConstants.*;

public final class Limelight implements Periodic, Loggable {
    @AutoLog
    public static class LimelightInputs {
        public Pose2d botPose;
        public boolean validTarget;
    }
    
    public enum LEDMode {
        kPipeline,
        kOff,
        kBlink,
        kOn
    }

    private final String tableName;
    private String logName;
    private LimelightInputsAutoLogged inputs;
    
    private static final Map<String, Limelight> limelightMap = new HashMap<>();

    private Limelight(String tableName) {
        if(tableName == "" || tableName == null) {
            tableName = "limelight";
        } else {
            tableName = "limelight-" + tableName;
        }

        this.tableName = tableName;
        String[] spl = this.tableName.split("-");

        this.logName = "";

        for(String s : spl) {
            this.logName += s.substring(0, 1).toUpperCase() + s.substring(1);
        }

        this.inputs = new LimelightInputsAutoLogged();

        PeriodicsUtil.registerPeriodic(this);
    }

    public double[] getBotPose() {
        double[] pose = LimelightHelpers.getBotPose(tableName);
        if(pose.length != 7) return new double[7];
        inputs.botPose = new Pose2d(pose[0] + FIELD_HALF_WIDTH_METERS, pose[1] + FIELD_HALF_HEIGHT_METERS, Rotation2d.fromDegrees(pose[5]));
        return pose;
    }

    public double[] getBotPoseInTargetSpace() {
        double[] pose = LimelightHelpers.getBotPose_TargetSpace(tableName);
        if(pose.length != 7) return new double[7];
        return pose;
    }

    public Pose2d getRobotPose() {
        getBotPose();
        
        return inputs.botPose;
    }

    public boolean isValidTarget() {
        return (inputs.validTarget = LimelightHelpers.getLimelightNTDouble(tableName, "tv") != 0.0);
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

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }

    @Override
    public void periodic() {
        
        log("Periodics", logName);
    }
}