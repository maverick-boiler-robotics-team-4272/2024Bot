package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class PeriodicsUtil {
    public static interface Periodic {
        public void periodic();
    }

    private static List<Periodic> periodics = new ArrayList<>();

    public static void registerPeriodic(Periodic periodic) {
        periodics.add(periodic);
    }

    public static void runPeriodics() {
        for(Periodic p : periodics) {
            p.periodic();
        }
    }
}
