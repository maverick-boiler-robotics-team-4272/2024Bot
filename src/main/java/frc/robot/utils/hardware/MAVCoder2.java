package frc.robot.utils.hardware;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.Loggable;

// Hardware
import com.revrobotics.*;

// Constants
import static frc.robot.constants.RobotConstants.MAVCoderConstants.*;

public class MAVCoder2 implements Loggable {
    @AutoLog
    public static class MAVCoder2Inputs {
        public double readAngleDegrees;
        public double offsetAngleDegrees;
    }

    private SparkAbsoluteEncoder encoder;
    private double offset;

    private MAVCoder2InputsAutoLogged inputs;

    public MAVCoder2(CANSparkBase motor, double offset) {
        this.encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.offset = offset;
        this.inputs = new MAVCoder2InputsAutoLogged();

        this.encoder.setZeroOffset(MAV_2_MIN_OUTPUT);
        this.encoder.setInverted(false);
        this.encoder.setPositionConversionFactor(MAV_2_POSITION_FACTOR);
    }

    public double getUnoffsetPosition() {
        return encoder.getPosition();
    }

    public double getPosition() {
        return encoder.getPosition() - offset;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        inputs.readAngleDegrees = getUnoffsetPosition();
        inputs.offsetAngleDegrees = inputs.readAngleDegrees - offset;

        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }
}
