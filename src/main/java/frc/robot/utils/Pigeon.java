package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4272.globals.Gyroscope;

public class Pigeon extends Pigeon2 implements Gyroscope, Loggable {

    @AutoLog
    public static class PigeonInputs {
        public double gyroscopeAngleRadians;
    }

    private PigeonInputsAutoLogged pigeonInputs;

    public Pigeon(int id) {
        super(id);

        pigeonInputs = new PigeonInputsAutoLogged();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(-getYaw().getValueAsDouble());
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        // TODO Auto-generated method stub
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        pigeonInputs.gyroscopeAngleRadians = getRotation().getRadians();

        Logger.processInputs(subdirectory + "/" + humanReadableName, pigeonInputs);
    }
}
