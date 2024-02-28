package frc.robot.constants;

import frc.robot.utils.paths.PositionsContainer;

import static edu.wpi.first.units.Units.*;

public class UniversalConstants {
    private UniversalConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static final double FIELD_WIDTH_METERS = Meters.convertFrom(54, Feet) + Meters.convertFrom(3.25, Inches);
    public static final double FIELD_HEIGHT_METERS = Meters.convertFrom(26, Feet) + Meters.convertFrom(11.25, Inches);

    public static final double FIELD_HALF_WIDTH_METERS = FIELD_WIDTH_METERS / 2.0;
    public static final double FIELD_HALF_HEIGHT_METERS = FIELD_HEIGHT_METERS / 2.0;

    public static final double PI2 = Math.PI * 2.0;

    public static final PositionsContainer RED_POSITIONS = new PositionsContainer("Red");
    public static final PositionsContainer BLUE_POSITIONS = new PositionsContainer("Blue");
    private static PositionsContainer globalPositions;

    public static void setGlobalPositions(PositionsContainer positions) {
        if(globalPositions != null)
            return;
        globalPositions = positions;
    }

    public static PositionsContainer getGlobalPositions() {
        return globalPositions;
    }

    public static boolean hasGlobalPositions() {
        return globalPositions != null;
    }
}
