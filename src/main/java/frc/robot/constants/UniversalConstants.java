package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;

public class UniversalConstants {
    private UniversalConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static final double FIELD_WIDTH_METERS = Units.Meters.convertFrom(54, Units.Feet) + Units.Meters.convertFrom(3.25, Units.Inches);
    public static final double FIELD_HEIGHT_METERS = Units.Meters.convertFrom(26, Units.Feet) + Units.Meters.convertFrom(11.25, Units.Inches);

    public static final Translation2d SPEAKER_POSITION = new Translation2d(0, Units.Meters.convertFrom(218.42, Units.Inches));

    public static final Pose2d AMP_POSE = new Pose2d(Units.Meters.convertFrom(72.5, Units.Inches), Units.Meters.convertFrom(300, Units.Inches), new Rotation2d(Math.PI / 2));

    public static final double FIELD_HALF_WIDTH_METERS = FIELD_WIDTH_METERS / 2.0;
    public static final double FIELD_HALF_HEIGHT_METERS = FIELD_HEIGHT_METERS / 2.0;

    public static final double PI2 = Math.PI * 2.0;
}
