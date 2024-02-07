package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
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
        public static final double MAX_TRANSLATIONAL_SPEED = 4.0;
        public static final double MAX_ROTATIONAL_SPEED = 2 * Math.PI;

        public static final Translation2d FRONT_LEFT_POSITION  = new Translation2d(-WHEEL_DISTANCE,  WHEEL_DISTANCE);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(-WHEEL_DISTANCE, -WHEEL_DISTANCE);
        public static final Translation2d BACK_LEFT_POSITION   = new Translation2d( WHEEL_DISTANCE,  WHEEL_DISTANCE);
        public static final Translation2d BACK_RIGHT_POSITION  = new Translation2d( WHEEL_DISTANCE, -WHEEL_DISTANCE);

        public static final double FRONT_LEFT_OFFSET  = 110.0;
        public static final double FRONT_RIGHT_OFFSET = 45.0;
        public static final double BACK_LEFT_OFFSET   = 164.0;
        public static final double BACK_RIGHT_OFFSET  = 316.0;

        public static class SwerveModuleConstants {
            private SwerveModuleConstants() {
                throw new UnsupportedOperationException("Cannot construct a constants class");
            }

            public static final double WHEEL_RADIUS = 2.0; // Inches
            public static final double MAX_MODULE_SPEED = Units.MetersPerSecond.convertFrom(14.5, Units.FeetPerSecond);

            public static final double DRIVE_RATIO = 6.75 / 1.0;
            public static final double STEER_RATIO = 150.0 / 7.0;

            public static final double DRIVE_PID_P = 0.003596;
            public static final double DRIVE_PID_I = 0.0;
            public static final double DRIVE_PID_D = 0.0;
            public static final double DRIVE_PID_F = 0.245;

            public static final double STEER_PID_P = 0.01;
            public static final double STEER_PID_I = 0.0001;
            public static final double STEER_PID_D = 0.0;
            public static final double STEER_PID_F = 0.0;
        }
    }

    public static interface ArmElevatorSetpoint {
        public double getElevatorHeight();
        public Rotation2d getArmAngle();

        public static ArmElevatorSetpoint createArbitrarySetpoint(double elevatorHeight, Rotation2d armAngle) {
            return new ArmElevatorSetpoint() {
                @Override
                public double getElevatorHeight() {
                    return elevatorHeight;
                }

                @Override
                public Rotation2d getArmAngle() {
                    return armAngle;
                }
            };
        }
    }

    public static enum ArmElevatorSetpoints implements ArmElevatorSetpoint {
        ZERO(new Rotation2d(0), 0);

        private Rotation2d armAngle;
        private double elevatorHeight;

        private ArmElevatorSetpoints(Rotation2d armAngle, double elevatorHeight) {
            this.armAngle = armAngle;
            this.elevatorHeight = elevatorHeight;
        }

        @Override
        public Rotation2d getArmAngle() {
            return armAngle;
        }

        @Override
        public double getElevatorHeight() {
            return elevatorHeight;
        }
    }

    public static class ArmConstants {
        public static final double ARM_PID_P = 0.001;
        public static final double ARM_PID_I = 0.0001;
        public static final double ARM_PID_D = 0.0;
        public static final double ARM_PID_F = 0.0;

        public static final double ARM_LENGTH = Units.Meters.convertFrom(16.0, Units.Inches);

        public static final double ARM_RATIO = 1.0;

        public static final Rotation2d MAX_ARM_ANGLE = Rotation2d.fromDegrees(50.0);
        public static final Rotation2d MIN_ARM_ANGLE = Rotation2d.fromDegrees(-20.0);
    }

    public static class ElevatorConstants {
        public static final double ELEVATOR_PID_P = 0.001;
        public static final double ELEVATOR_PID_I = 0.0001;
        public static final double ELEVATOR_PID_D = 0.0;
        public static final double ELEVATOR_PID_F = 0.0;

        public static final double BLOCKING_HEIGHT = Units.Meters.convertFrom(3.0, Units.Inches);

        public static final double ELEVATOR_RATIO = 1.0; //Find empericaly

        //TODO: find out from CAD
        public static final double MAX_ELEVATOR_HEIGHT = Units.Meters.convertFrom(48.0, Units.Inches);
        public static final double MIN_ELEVATOR_HEIGHT = Units.Meters.convertFrom(0, Units.Inches);
    }
}
