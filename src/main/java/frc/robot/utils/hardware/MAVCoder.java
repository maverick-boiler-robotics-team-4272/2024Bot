package frc.robot.utils.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;

import frc.robot.utils.logging.Loggable;

public class MAVCoder implements Loggable {
    
    @AutoLog
    public static class MAVCoderInputs {
        public double readVoltage;
        public double readAngleDegrees;
        public double offsetAngleDegrees;
    };

    private SparkAnalogSensor sensor;
    private double offset;

    private MAVCoderInputsAutoLogged mavCoderInputs;

    public MAVCoder(NEO motor, double offset) {
        this.sensor = motor.getAnalog(Mode.kAbsolute);
        this.offset = offset;

        this.mavCoderInputs = new MAVCoderInputsAutoLogged();
    }

    public double getVoltageReading() {
        return sensor.getPosition();
    }

    public double getUnoffsetAngle() {
        return sensor.getPosition() * 360.0 / 3.3;
    }

    public double getAngle() {
        return sensor.getPosition() * 360.0 / 3.3 - offset;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        mavCoderInputs.readVoltage = getVoltageReading();
        mavCoderInputs.readAngleDegrees = getUnoffsetAngle();
        mavCoderInputs.offsetAngleDegrees = getAngle();

        Logger.processInputs(subdirectory + "/" + humanReadableName, mavCoderInputs);
    }
}
