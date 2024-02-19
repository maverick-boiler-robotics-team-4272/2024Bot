package frc.robot.utils;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.utils.PeriodicsUtil.Periodic;

public class Norm<T> implements Periodic, Loggable {
    @AutoLog
    public static class NormInputs {
        public double n1Norm;
        public double n1NormNormalized;
        
        public double n2Norm;
        public double n2NormNormalized;

        public double nInfNorm;
        public double nInfNormNormalized;

        public int count;

        public boolean enabled;
    }

    public static interface Metric<T> {
        public double getDistance(T a, T b);
    }

    private NormInputsAutoLogged inputs;
    private String name;
    private Supplier<T> a;
    private Supplier<T> b;
    private Metric<T> metric;

    public Norm(Supplier<T> a, Supplier<T> b, Metric<T> metric, String name) {
        PeriodicsUtil.registerPeriodic(this);

        this.a = a;
        this.b = b;
        this.metric = metric;
        this.name = name;
        this.inputs = new NormInputsAutoLogged();
    }

    public void enable() {
        inputs.enabled = true;
    }

    public void disable() {
        inputs.enabled = false;
    }

    public void reset() {
        inputs.n1Norm = 0;
        inputs.n1NormNormalized = 0;
        inputs.n2Norm = 0;
        inputs.n2NormNormalized = 0;
        inputs.nInfNorm = 0;
        inputs.nInfNormNormalized = 0;

        inputs.count = 0;
    }

    private void updateNorms() {
        double dist = metric.getDistance(a.get(), b.get());
        
        inputs.count++;

        inputs.n1Norm += dist;
        inputs.n2Norm = Math.hypot(inputs.n2Norm, dist);
        inputs.nInfNorm = Math.max(inputs.nInfNorm, dist);

        inputs.n1NormNormalized = inputs.n1Norm / inputs.count;
        inputs.n2NormNormalized = inputs.n2Norm / Math.sqrt(inputs.count);
        inputs.nInfNormNormalized = inputs.nInfNorm; // Already asymptotic to 1, just have the extra slot for completeness sake
    }
    
    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }
    
    @Override
    public void periodic() {
        if(inputs.enabled)
            updateNorms();
        log("Periodics", name);
    }
}
