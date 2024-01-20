package frc.robot.constants;

import frc.robot.limelight.Limelight;
import frc.robot.utils.ShuffleboardTable;

public class TelemetryConstants {
    private TelemetryConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static class Limelights {
        private Limelights() {
            throw new UnsupportedOperationException("Cannot construct a constants class");
        }
        public static final Limelight CENTER_LIMELIGHT = Limelight.getLimelight("center");
    }

    public static class ShuffleboardTables {
        private ShuffleboardTables() {
            throw new UnsupportedOperationException("Cannot construct a constants class");
        }

        public static final ShuffleboardTable AUTO_TABLE = ShuffleboardTable.getTable("Auto");
    }
}
