package frc.robot.utils.periodics;

import java.util.*;

public class PeriodicsUtil {
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
