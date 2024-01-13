package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;

public class RobotConstants {
    private RobotConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static final int NOMINAL_VOLTAGE = 11;
    public static final int CURRENT_LIMIT = 20;

    public static class DrivetrainConstants {
        private DrivetrainConstants() {
            throw new UnsupportedOperationException("Cannot construct a constants class");
        }

        public static final double WHEEL_DISTANCE = Units.Meters.convertFrom(1, Units.Feet);

        public static final Translation2d FRONT_LEFT_POSITION  = new Translation2d(-WHEEL_DISTANCE,  WHEEL_DISTANCE);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(-WHEEL_DISTANCE, -WHEEL_DISTANCE);
        public static final Translation2d BACK_LEFT_POSITION   = new Translation2d( WHEEL_DISTANCE,  WHEEL_DISTANCE);
        public static final Translation2d BACK_RIGHT_POSITION  = new Translation2d( WHEEL_DISTANCE, -WHEEL_DISTANCE);

        public static final double FRONT_LEFT_OFFSET  = 110.0;       
        public static final double FRONT_RIGHT_OFFSET = 269.0;
        public static final double BACK_LEFT_OFFSET   = 164.0;
        public static final double BACK_RIGHT_OFFSET  = 316.0;

        public static class SwerveModuleConstants {
            private SwerveModuleConstants() {
                throw new UnsupportedOperationException("Cannot construct a constants class");
            }

            public static final double WHEEL_RADIUS = 2.0; // Inches

            public static final double DRIVE_RATIO = 6.75 / 1.0;
            public static final double STEER_RATIO = 150.0 / 7.0;

            public static final double DRIVE_PID_P = 0.003596;
            public static final double DRIVE_PID_I = 0.0;
            public static final double DRIVE_PID_D = 0.0;
            public static final double DRIVE_PID_F = 0.6;

            public static final double STEER_PID_P = 0.01;
            public static final double STEER_PID_I = 0.0001;
            public static final double STEER_PID_D = 0.0;
            public static final double STEER_PID_F = 0.0;
        }
    }
}
