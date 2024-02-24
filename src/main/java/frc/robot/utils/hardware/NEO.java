package frc.robot.utils.hardware;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import frc.robot.utils.logging.Loggable;

public class NEO extends CANSparkMax implements Loggable {
    private MotorInputsAutoLogged motorInputs;
    private RelativeEncoder encoder;

    public NEO(int canId) {
        super(canId, MotorType.kBrushless);

        motorInputs = new MotorInputsAutoLogged();
        encoder = getEncoder();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        motorInputs.motorVoltage = getBusVoltage();
        // motorInputs.outputCurrent = getOutputCurrent();
        motorInputs.motorVelocity = encoder.getVelocity();
        motorInputs.motorPosition = encoder.getPosition();
        // motorInputs.stalling = motorInputs.outputCurrent >= motorInputs.currentLimit * 0.9 && motorInputs.motorVelocity / encoder.getVelocityConversionFactor() < 100;
        // motorInputs.motorTemperatureCelsius = getMotorTemperature();

        Logger.processInputs(subdirectory + "/" + humanReadableName, motorInputs);
    }

    public boolean isStalled() {
        // return motorInputs.stalling;
        return false;
    }

    @Override
    public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
        motorInputs.currentLimit = stallLimit;
        motorInputs.freeLimit = freeLimit;
        motorInputs.limitRPM = limitRPM;
        
        return super.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    }
}
