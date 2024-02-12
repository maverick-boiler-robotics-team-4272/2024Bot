package frc.robot.utils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import static frc.robot.constants.RobotConstants.*;

public class NEOBuilder {
    private NEO motor;
    private RelativeEncoder motorEncoder;
    private SparkPIDController motorController;

    private NEOBuilder(int id) {
        motor = new NEO(id);

        motor.restoreFactoryDefaults();
    }

    public NEOBuilder withVoltageCompensation(int nominalVoltage) {
        motor.enableVoltageCompensation(nominalVoltage);

        return this;
    }

    public NEOBuilder withCurrentLimit(int currentLimit) {
        motor.setSmartCurrentLimit(currentLimit);

        return this;
    }

    public NEOBuilder withIdleMode(IdleMode mode) {
        motor.setIdleMode(mode);

        return this;
    }

    public NEOBuilder withInversion(boolean inverted) {
        motor.setInverted(inverted);

        return this;
    }

    public NEOBuilder withPIDParams(double p, double i, double d) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setP(p);
        motorController.setI(i);
        motorController.setD(d);

        return this;
    }

    public NEOBuilder withPIDFParams(double p, double i, double d, double f) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setP(p);
        motorController.setI(i);
        motorController.setD(d);
        motorController.setFF(f);

        return this;
    }

    public NEOBuilder withPositionConversionFactor(double factor) {
        if(motorEncoder == null)
            motorEncoder = motor.getEncoder();
        
        motorEncoder.setPositionConversionFactor(factor);

        return this;
    }

    public NEOBuilder withPosition(double position) {
        if(motorEncoder == null)
            motorEncoder = motor.getEncoder();

        motorEncoder.setPosition(position);

        return this;
    }

    public NEOBuilder withVelocityConversionFactor(double factor) {
        if(motorEncoder == null)
            motorEncoder = motor.getEncoder();

        motorEncoder.setVelocityConversionFactor(factor);

        return this;
    }

    public NEOBuilder withForwardSoftlimit(double limit) {
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float)limit);

        return this;
    }

    public NEOBuilder withReverseSoftLimit(double limit) {
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)limit);

        return this;
    }

    public NEOBuilder withSoftLimits(double forwardLimit, double reverseLimit) {
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        motor.setSoftLimit(SoftLimitDirection.kForward, (float)forwardLimit);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)reverseLimit);

        return this;
    }

    public NEOBuilder withOutputRange(double min, double max) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setOutputRange(min, max);

        return this;
    }

    public NEOBuilder withClosedLoopRampRate(double rate) {
        motor.setClosedLoopRampRate(rate);

        return this;
    }

    public NEOBuilder withOpenLoopRampRate(double rate) {
        motor.setOpenLoopRampRate(rate);

        return this;
    }

    public NEOBuilder withPIDPositionWrapping(double min, double max) {
        if(motorController == null)
            motorController = motor.getPIDController();

        motorController.setPositionPIDWrappingEnabled(true);
        motorController.setPositionPIDWrappingMaxInput(max);
        motorController.setPositionPIDWrappingMinInput(min);

        return this;
    }

    public NEO build() {
        motor.burnFlash();
        return motor;
    }

    public NEO getUnburntNeo() {
        return motor;
    }

    public static NEOBuilder createWithDefaults(int id) {
        return new NEOBuilder(id)
            .withCurrentLimit(CURRENT_LIMIT)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false);
    }

    public static NEOBuilder create(int id) {
        return new NEOBuilder(id);
    }
}
