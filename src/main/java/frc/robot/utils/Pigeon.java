package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4272.globals.Gyroscope;
import frc.team4272.globals.MathUtils;

public class Pigeon extends Pigeon2 implements Gyroscope, Loggable {

    @AutoLog
    public static class PigeonInputs {
        public double gyroscopeAngleDegrees;
        public double outputAngleDegrees;
    }

    private PigeonInputsAutoLogged pigeonInputs;
    private double offset;

    public Pigeon(int id) {
        super(id);

        pigeonInputs = new PigeonInputsAutoLogged();

        setYaw(MathUtils.euclideanModulo(getYaw().getValueAsDouble(), 360.0));
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(-getYaw().getValueAsDouble() + offset);
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        offset = getYaw().getValueAsDouble() - rotation.getDegrees();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        pigeonInputs.gyroscopeAngleDegrees = getYaw().getValueAsDouble();
        pigeonInputs.outputAngleDegrees = -getYaw().getValueAsDouble() + offset;

        Logger.processInputs(subdirectory + "/" + humanReadableName, pigeonInputs);
    }
}
