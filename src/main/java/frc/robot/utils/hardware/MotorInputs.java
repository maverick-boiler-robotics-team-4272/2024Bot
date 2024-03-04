package frc.robot.utils.hardware;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs {
    public double motorVoltage;
    // public double motorTemperatureCelsius;
    public double outputCurrent;
    public boolean stalling;

    public double currentLimit;
    public double freeLimit;
    public double limitRPM;

    public double motorVelocity;
    public double motorPosition;
}
