package frc.robot.utils.hardware;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.Loggable;

// Hardware
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

    public double getRangeMeters() {
        return getRange() / 1000.0;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        lidarInputs.lidarDistance = getRange();

        Logger.processInputs(subdirectory + "/" + humanReadableName, lidarInputs);
    }
}
