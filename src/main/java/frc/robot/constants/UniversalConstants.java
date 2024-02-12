package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.*;

public class UniversalConstants {
    private UniversalConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static final double FIELD_WIDTH_METERS = Meters.convertFrom(54, Feet) + Meters.convertFrom(3.25, Inches);
    public static final double FIELD_HEIGHT_METERS = Meters.convertFrom(26, Feet) + Meters.convertFrom(11.25, Inches);

    public static final Translation2d SPEAKER_POSITION = new Translation2d(0, Meters.convertFrom(218.42, Inches));
    public static final Translation3d SPEAKER_SHOT_POSITION = new Translation3d(Meters.convertFrom(10, Inches), Meters.convertFrom(218.42, Inches), Meters.convertFrom(6.66667, Feet));

    public static final Pose2d AMP_POSE = new Pose2d(Meters.convertFrom(72.5, Inches), Meters.convertFrom(300, Inches), new Rotation2d(Math.PI / 2));

    public static final double FIELD_HALF_WIDTH_METERS = FIELD_WIDTH_METERS / 2.0;
    public static final double FIELD_HALF_HEIGHT_METERS = FIELD_HEIGHT_METERS / 2.0;

    public static final double PI2 = Math.PI * 2.0;
}
