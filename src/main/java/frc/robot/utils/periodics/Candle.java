package frc.robot.utils.periodics;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;

import frc.robot.utils.logging.Loggable;

public class Candle extends CANdle implements Periodic, Loggable {
    @AutoLog
    public static class CandleInputs {

    }
    
    private CandleInputsAutoLogged inputs;
    public Candle(int id) {
        super(id);

        inputs = new CandleInputsAutoLogged();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }

    @Override
    public void periodic() {
        log("Periodics", "Candle");
    }
}
