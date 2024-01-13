package frc.robot.utils;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import static frc.robot.constants.RobotConstants.*;

public class VortexBuilder {
    private Vortex motor;
    private SparkPIDController motorController;
    private RelativeEncoder motorEncoder;

    private VortexBuilder(int id) {
        motor = new Vortex(id);
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
