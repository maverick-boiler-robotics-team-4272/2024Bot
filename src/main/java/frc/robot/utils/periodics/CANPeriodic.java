package frc.robot.utils.periodics;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.logging.Loggable;

public class CANPeriodic implements Periodic, Loggable {
    private static class AverageFilter {
        private DoubleCircularBuffer data;

        public AverageFilter(int size) {
            data = new DoubleCircularBuffer(size);
        }

        public double calculate(double input) {
            data.addFirst(input);

            double sum = 0;
            int len = data.size();

            for(int i = 0; i < len; i++)
                sum += data.get(i);
            return sum / len;
        }
    }

    @AutoLog
    public static class CANInputs {
        public double canUsage;
        public double averagedUsage;
    }

    private AverageFilter filter;
    private CANInputsAutoLogged inputs;

    private CANPeriodic() {
        filter = new AverageFilter(10);
        inputs = new CANInputsAutoLogged();

        PeriodicsUtil.registerPeriodic(this);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        inputs.canUsage = RobotController.getCANStatus().percentBusUtilization;
        inputs.averagedUsage = filter.calculate(inputs.canUsage);

        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }

    @Override
    public void periodic() {
        log("Periodics", "CANUsage");
    }

    public static void setUpLogging() {
        new CANPeriodic();
    }
}
