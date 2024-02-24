package frc.robot.utils.hardware;

// Logging
import org.littletonrobotics.junction.Logger;
import frc.robot.utils.logging.Loggable;

// Hardware
import com.revrobotics.*;

public class Vortex extends CANSparkFlex implements Loggable {
    private MotorInputsAutoLogged motorInputs;
    private RelativeEncoder encoder;

    public Vortex(int id) {
        super(id, MotorType.kBrushless);

        motorInputs = new MotorInputsAutoLogged();
        this.encoder = getEncoder();
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

    public boolean isStalling() {
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
