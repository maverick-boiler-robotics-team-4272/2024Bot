package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

public class NEO extends CANSparkMax implements Loggable {
    @AutoLog
    public static class NEOInputs {
        public double motorVoltage;
        public double outputCurrent;
    }

    NEOInputsAutoLogged neoInputs;

    public NEO(int canId) {
        super(canId, MotorType.kBrushless);

        neoInputs = new NEOInputsAutoLogged();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        neoInputs.motorVoltage = getBusVoltage();
        neoInputs.outputCurrent = getOutputCurrent();

        Logger.processInputs(subdirectory + "/" + humanReadableName, neoInputs);
    }
}
