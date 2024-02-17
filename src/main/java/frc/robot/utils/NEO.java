package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

public class NEO extends CANSparkMax implements Loggable {
    @AutoLog
    public static class NEOInputs {
        public double motorVoltage;
    }

    NEOInputsAutoLogged neoInputs;

    public NEO(int canId) {
        super(canId, MotorType.kBrushless);

        neoInputs = new NEOInputsAutoLogged();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        neoInputs.motorVoltage = getBusVoltage();

        Logger.processInputs(subdirectory + "/" + humanReadableName, neoInputs);
    }
}
