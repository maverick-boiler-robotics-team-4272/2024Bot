package frc.robot.utils.hardware;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.Loggable;

// Hardware
import frc.team4272.globals.Gyroscope;
import com.ctre.phoenix6.hardware.Pigeon2;

// Math
import frc.team4272.globals.MathUtils;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends Pigeon2 implements Gyroscope, Loggable {

    @AutoLog
    public static class PigeonInputs {
        public double gyroscopeAngleDegrees;
        public double outputAngleDegrees;

        public double xAcceleration;
        public double yAcceleration;
        public double zAcceleration;

        public Rotation2d xyAccelerationAngle;
        public double xyAccelerationMagnitude;
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
        pigeonInputs.xAcceleration = getAccelerationX().getValueAsDouble();
        pigeonInputs.yAcceleration = getAccelerationY().getValueAsDouble();
        pigeonInputs.zAcceleration = getAccelerationZ().getValueAsDouble();

        pigeonInputs.xyAccelerationAngle = new Rotation2d(pigeonInputs.yAcceleration, pigeonInputs.xAcceleration);
        pigeonInputs.xyAccelerationMagnitude = Math.hypot(pigeonInputs.yAcceleration, pigeonInputs.xAcceleration);

        Logger.processInputs(subdirectory + "/" + humanReadableName, pigeonInputs);
    }
}
