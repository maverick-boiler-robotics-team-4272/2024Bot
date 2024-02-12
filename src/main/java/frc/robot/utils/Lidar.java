package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

public class Lidar extends TimeOfFlight implements Loggable {
    @AutoLog
    public static class LidarInputs {
        public double lidarDistance;
    }

    private LidarInputsAutoLogged lidarInputs = new LidarInputsAutoLogged();

    public Lidar(int id) {
        super(id);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        lidarInputs.lidarDistance = getRange();

        Logger.processInputs(subdirectory + "/" + humanReadableName, lidarInputs);
    }
}
