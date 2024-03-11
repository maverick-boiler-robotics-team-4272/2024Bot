package frc.robot.utils.hardware;

// Logging
import org.littletonrobotics.junction.*;
import frc.robot.utils.logging.Loggable;

// Hardware
import com.revrobotics.*;
import com.revrobotics.jni.CANSparkMaxJNI;

// Constants
import static frc.robot.constants.RobotConstants.MAVCoderConstants.*;

import java.lang.reflect.Field;

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

        Class<CANSparkLowLevel> clazz = CANSparkLowLevel.class;

        long handle = 0;

        try {
            Field id = clazz.getDeclaredField("sparkMaxHandle");

            id.setAccessible(true);

            handle = (Long)id.get(motor);

            id.setAccessible(false);
        } catch(Exception e) {
            throw new RuntimeException("Could not get handle", e);
        }

        CANSparkMaxJNI.c_SparkMax_SetFeedbackDeviceRange(handle, 0, 360);
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
