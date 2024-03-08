package frc.robot.utils.misc;

import java.util.*;

import edu.wpi.first.math.Pair;

public class InterpolationMap {
    private List<Pair<Double, Double>> interpolants = new ArrayList<>();

    public InterpolationMap(Pair<Double, Double>[] dataPoints) {
        interpolants = new ArrayList<>();

        for(var d : dataPoints)
            interpolants.add(d);
    }

    public double getInterpolatedValue(double measured) {
        int n = 1;

        for(; n < interpolants.size(); n++) {
            if(measured > interpolants.get(n).getFirst()) {
                break;
            }
        }

        double measured0 = interpolants.get(n - 1).getFirst();
        double measured1 = interpolants.get(n).getFirst();

        double output0 = interpolants.get(n - 1).getSecond();
        double output1 = interpolants.get(n).getSecond();

        double t = (measured - measured0) / (measured1 - measured0);

        return output0 + (output1 - output0) * t;
    }
}
