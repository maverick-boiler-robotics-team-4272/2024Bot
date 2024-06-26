package frc.robot.limelight;

import java.util.*;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.*;

// Math
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.*;

// Periodics
import frc.robot.utils.periodics.*;

// Constants
import static frc.robot.constants.UniversalConstants.*;

public final class Limelight implements Periodic, Loggable {
    @AutoLog
    public static class LimelightInputs {
        public Pose2d unfilteredPose;
        public Pose2d filteredPose;
        public boolean validTarget;
    }
    
    public enum LEDMode {
        kPipeline,
        kOff,
        kBlink,
        kOn
    }

    private static final int ARRAY_LENGTH = 25;

    private final String tableName;
    private String logName;
    private LimelightInputsAutoLogged inputs;
    private boolean updatedInputs = false;
    private boolean filtered = false;
    private double[] updateCycleRobotPose = new double[ARRAY_LENGTH];

    private static final int FILTER_SIZE = 11;

    private MedianFilter xPositionFilter = new MedianFilter(FILTER_SIZE);
    private MedianFilter yPositionFilter = new MedianFilter(FILTER_SIZE);
    private MedianFilter thetaDegsFilter = new MedianFilter(FILTER_SIZE);

    
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
        this.inputs.filteredPose = new Pose2d();
        this.inputs.unfilteredPose =  new Pose2d();

        PeriodicsUtil.registerPeriodic(this);
    }

    public double[] getBotPose() {
        double[] pose = LimelightHelpers.getBotPose(tableName);
        if(pose.length != ARRAY_LENGTH) return new double[ARRAY_LENGTH];
        if(!updatedInputs) {
            for(int i = 0; i < ARRAY_LENGTH; i++)
                updateCycleRobotPose[i] = pose[i];

            inputs.unfilteredPose = new Pose2d(pose[0] + FIELD_HALF_WIDTH_METERS, pose[1] + FIELD_HALF_HEIGHT_METERS, Rotation2d.fromDegrees(pose[5]));
            updatedInputs = true;
        }

        return updateCycleRobotPose;
    }

    public double[] getBotPoseInTargetSpace() {
        double[] pose = LimelightHelpers.getBotPose_TargetSpace(tableName);
        if(pose.length != ARRAY_LENGTH) return new double[ARRAY_LENGTH];
        return pose;
    }

    public Pose2d getRobotPose() {
        filterPosition();
        return inputs.filteredPose;
    }

    public Pose2d getUnfilteredPose() {
        getBotPose();

        return inputs.unfilteredPose;
    }

    public void filterPosition() {
        if(filtered)
            return;
        
        filtered = true;
        double[] pose = getBotPose();

        if(pose[0] == 0 && pose[1] == 0 && pose[5] == 0) {
            // Filler for invalid
            xPositionFilter.reset();
            yPositionFilter.reset();
            thetaDegsFilter.reset();

            inputs.filteredPose = new Pose2d(FIELD_HALF_WIDTH_METERS, FIELD_HALF_HEIGHT_METERS, new Rotation2d(0));
            return;
        }

        inputs.filteredPose = new Pose2d(
            xPositionFilter.calculate(pose[0]) + FIELD_HALF_WIDTH_METERS,
            yPositionFilter.calculate(pose[1]) + FIELD_HALF_HEIGHT_METERS,
            Rotation2d.fromDegrees(thetaDegsFilter.calculate(pose[5]))
        );

    }

    public boolean isValidTarget() {
        double[] pose = getBotPose();

        return (inputs.validTarget = !(pose[0] == 0 && pose[1] == 0 && pose[5] == 0));
    }

    public boolean getTV() {
        return LimelightHelpers.getTV(tableName);
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
        filterPosition();
        log("Periodics", logName);
        
        updatedInputs = false;
        filtered = false;
    }
}