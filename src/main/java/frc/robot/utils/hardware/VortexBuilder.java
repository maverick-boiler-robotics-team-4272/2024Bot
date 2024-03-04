package frc.robot.utils.hardware;

// Hardware
import com.revrobotics.CANSparkBase.*;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.*;

// Constants
import static frc.robot.constants.RobotConstants.*;

public class VortexBuilder {
    private Vortex motor;
    private SparkPIDController motorController;
    private RelativeEncoder motorEncoder;

    private VortexBuilder(int id) {
        motor = new Vortex(id);
        motor.restoreFactoryDefaults();
    }

    public VortexBuilder asFollower(Vortex motor, boolean inverted) {
        this.motor.follow(motor, inverted);

        return this;
    }

    public VortexBuilder withVoltageCompensation(int nominalVoltage) {
        motor.enableVoltageCompensation(nominalVoltage);

        return this;
    }

    public VortexBuilder withCurrentLimit(int currentLimit) {
        motor.setSmartCurrentLimit(currentLimit);

        return this;
    }

    public VortexBuilder withIdleMode(IdleMode mode) {
        motor.setIdleMode(mode);

        return this;
    }

    public VortexBuilder withInversion(boolean inverted) {
        motor.setInverted(inverted);

        return this;
    }

    public VortexBuilder withPIDParams(double p, double i, double d) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setP(p);
        motorController.setI(i);
        motorController.setD(d);

        return this;
    }

    public VortexBuilder withPIDFParams(double p, double i, double d, double f) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setP(p);
        motorController.setI(i);
        motorController.setD(d);
        motorController.setFF(f);

        return this;
    }

    public VortexBuilder withPIDClamping(double min, double max) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setOutputRange(min, max);

        return this;
    }

    public VortexBuilder withMaxIAccum(double max) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setIMaxAccum(max, 0);
        return this;
    }

    public VortexBuilder withPositionConversionFactor(double factor) {
        if(motorEncoder == null)
            motorEncoder = motor.getEncoder();
        
        motorEncoder.setPositionConversionFactor(factor);

        return this;
    }

    public VortexBuilder withPosition(double position) {
        if(motorEncoder == null)
            motorEncoder = motor.getEncoder();

        motorEncoder.setPosition(position);

        return this;
    }

    public VortexBuilder withVelocityConversionFactor(double factor) {
        if(motorEncoder == null)
            motorEncoder = motor.getEncoder();

        motorEncoder.setVelocityConversionFactor(factor);

        return this;
    }

    public VortexBuilder withForwardSoftlimit(double limit) {
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float)limit);

        return this;
    }

    public VortexBuilder withReverseSoftLimit(double limit) {
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)limit);

        return this;
    }

    public VortexBuilder withSoftLimits(double forwardLimit, double reverseLimit) {
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        motor.setSoftLimit(SoftLimitDirection.kForward, (float)forwardLimit);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)reverseLimit);

        return this;
    }

    public VortexBuilder withOutputRange(double min, double max) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setOutputRange(min, max);

        return this;
    }

    public VortexBuilder withClosedLoopRampRate(double rate) {
        motor.setClosedLoopRampRate(rate);

        return this;
    }

    public VortexBuilder withOpenLoopRampRate(double rate) {
        motor.setOpenLoopRampRate(rate);

        return this;
    }

    public VortexBuilder withPIDPositionWrapping(double min, double max) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setPositionPIDWrappingEnabled(true);
        motorController.setPositionPIDWrappingMaxInput(max);
        motorController.setPositionPIDWrappingMinInput(min);

        return this;
    }

    /**
     * 
     * 
     * <p>Status Frame 0: Applied Set, Faults, Sticky Faults, Is Follower, Default: 10ms</p>
     * <p>Status Frame 1: Motor Velocity, Motor Temperature, Motor Voltage, Motor Current, Default: 20ms</p>
     * <p>Status Frame 2: Motor Position, Default: 20ms</p>
     * <p>Status Frame 3: Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position, Default: 50ms</p>
     * <p>Status Frame 4: Alternate Encoder Velocity, Alternate Encoder Position, Default: 20ms</p>
     * <p>Status Frame 5: Duty Cycle Absolute Encoder Position, Default: 200ms</p>
     * <p>Status Frame 6: Duty Cycle Absolute Encoder Velocity, Default: 500ms</p>
     * @param frame which frame you wish to set
     * @param ms period time in milliseconds
     */
    public VortexBuilder withPeriodicFramerate(PeriodicFrame frame, int ms) {
        motor.setPeriodicFramePeriod(frame, ms);

        return this;
    }

    public Vortex build() {
        motor.burnFlash();
        return motor;
    }

    public Vortex getUnburntNeo() {
        return motor;
    }

    public static VortexBuilder createWithDefaults(int id) {
        return new VortexBuilder(id)
            .withCurrentLimit(CURRENT_LIMIT)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false);
    }

    public static VortexBuilder create(int id) {
        return new VortexBuilder(id);
    }
}
