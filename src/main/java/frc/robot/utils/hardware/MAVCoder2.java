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
    }

    private SparkAbsoluteEncoder encoder;

    private MAVCoder2InputsAutoLogged inputs;

    public MAVCoder2(CANSparkBase motor, double offset) {
        this.encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.inputs = new MAVCoder2InputsAutoLogged();

        this.encoder.setPositionConversionFactor(MAV_2_POSITION_FACTOR);
        this.encoder.setZeroOffset(MAV_2_ANGLE_OFFSET + offset);
        this.encoder.setInverted(true);
    }

    public SparkAbsoluteEncoder getEncoder() {
        return encoder;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        inputs.readAngleDegrees = getPosition();

        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }
}
