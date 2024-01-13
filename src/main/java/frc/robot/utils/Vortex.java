package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;

public class Vortex extends CANSparkFlex implements Loggable {

    @AutoLog
    public static class VortexInputs {
        public double motorVoltage;
    }

    private VortexInputsAutoLogged vortexInputs;

    public Vortex(int id) {
        super(id, MotorType.kBrushless);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        vortexInputs.motorVoltage = getBusVoltage();

        Logger.processInputs(subdirectory + "/" + humanReadableName, vortexInputs);
    }
}
