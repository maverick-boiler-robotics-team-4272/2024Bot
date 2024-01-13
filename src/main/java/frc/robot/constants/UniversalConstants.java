package frc.robot.constants;

import edu.wpi.first.units.Units;

public class UniversalConstants {
    private UniversalConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static final double FIELD_WIDTH_METERS = Units.Meters.convertFrom(54, Units.Feet) + Units.Meters.convertFrom(3.25, Units.Inches);
    public static final double FIELD_HEIGHT_METERS = Units.Meters.convertFrom(26, Units.Feet) + Units.Meters.convertFrom(11.25, Units.Inches);

    public static final double FIELD_HALF_WIDTH_METERS = FIELD_WIDTH_METERS / 2.0;
    public static final double FIELD_HALF_HEIGHT_METERS = FIELD_HEIGHT_METERS / 2.0;

    public static final double PI2 = Math.PI * 2.0;
}
