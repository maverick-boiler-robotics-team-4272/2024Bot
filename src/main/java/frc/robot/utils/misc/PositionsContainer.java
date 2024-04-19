package frc.robot.utils.misc;

import edu.wpi.first.math.geometry.*;

import static frc.robot.constants.UniversalConstants.*;
import static edu.wpi.first.units.Units.*;

public class PositionsContainer {
    public final Translation2d SPEAKER_POSITION;
    public final Translation3d SPEAKER_SHOT_POSITION;
    public final Translation3d SPEAKER_TARGET_POSITION;
    public final Pose2d AMP_POSE;
    public final Rotation2d TO_DRIVERSTATION;
    public final Pose2d TRAP_STAGE_POSE;

    public final Rotation2d TO_SOURCE;

    public PositionsContainer(String side) {
        boolean red = side.equals("Red");

        SPEAKER_POSITION = new Translation2d(red ? FIELD_WIDTH_METERS : 0, Meters.convertFrom(218.42, Inches));
        SPEAKER_SHOT_POSITION = new Translation3d(red ? FIELD_WIDTH_METERS - Meters.convertFrom(10, Inches) : Meters.convertFrom(10, Inches), Meters.convertFrom(218.42, Inches), Meters.convertFrom(6.7917, Feet));
        SPEAKER_TARGET_POSITION = new Translation3d(SPEAKER_POSITION.getX() + Meters.convertFrom(2.5, Inches), SPEAKER_POSITION.getY() - Meters.convertFrom(2.5, Inches), SPEAKER_SHOT_POSITION.getZ());
        AMP_POSE = new Pose2d(red ? FIELD_WIDTH_METERS - Meters.convertFrom(72.5, Inches) : Meters.convertFrom(72.5, Inches), Meters.convertFrom(300, Inches), new Rotation2d(Math.PI / 2));;
        TRAP_STAGE_POSE = new Pose2d(red ? FIELD_WIDTH_METERS - 4.20 : 4.20, 2.90, Rotation2d.fromDegrees(red ? 120 : 60));
        TO_DRIVERSTATION = Rotation2d.fromDegrees(red ? 0 : 180);

        TO_SOURCE = Rotation2d.fromDegrees(red ? 60.0 : 120.0);
    }
}
