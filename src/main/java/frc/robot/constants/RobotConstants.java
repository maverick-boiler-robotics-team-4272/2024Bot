package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.UniversalConstants.PI2;

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
        public static final double DRIVEBASE_HALF_WIDTH = Meters.convertFrom(9.875, Inches);
        public static final double DRIVEBASE_HALF_HEIGHT = Meters.convertFrom(8.875, Inches);
        public static final double MAX_TRANSLATIONAL_SPEED = 4.0;
        public static final double MAX_ROTATIONAL_SPEED = 2 * Math.PI;

        public static final Translation2d FRONT_LEFT_POSITION  = new Translation2d(-DRIVEBASE_HALF_WIDTH,  DRIVEBASE_HALF_HEIGHT);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(-DRIVEBASE_HALF_WIDTH, -DRIVEBASE_HALF_HEIGHT);
        public static final Translation2d BACK_LEFT_POSITION   = new Translation2d( DRIVEBASE_HALF_WIDTH,  DRIVEBASE_HALF_HEIGHT);
        public static final Translation2d BACK_RIGHT_POSITION  = new Translation2d( DRIVEBASE_HALF_WIDTH, -DRIVEBASE_HALF_HEIGHT);

        public static final double WHEEL_DISTANCE = Meters.convertFrom(1, Feet);

        // public static final Translation2d FRONT_LEFT_POSITION  = new Translation2d(-WHEEL_DISTANCE,  WHEEL_DISTANCE);
        // public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(-WHEEL_DISTANCE, -WHEEL_DISTANCE);
        // public static final Translation2d BACK_LEFT_POSITION   = new Translation2d( WHEEL_DISTANCE,  WHEEL_DISTANCE);
        // public static final Translation2d BACK_RIGHT_POSITION  = new Translation2d( WHEEL_DISTANCE, -WHEEL_DISTANCE);

        public static final double FRONT_LEFT_OFFSET  = 157.0;
        public static final double FRONT_RIGHT_OFFSET =   6.0;
        public static final double BACK_LEFT_OFFSET   = 250.0;
        public static final double BACK_RIGHT_OFFSET  = 310.0;
        
        public static class SwerveModuleConstants {
            private SwerveModuleConstants() {
                throw new UnsupportedOperationException("Cannot construct a constants class");
            }

            public static final double WHEEL_RADIUS = 2.0; // Inches
            public static final double MAX_MODULE_SPEED = MetersPerSecond.convertFrom(14.5, FeetPerSecond);

            public static final double DRIVE_RATIO = 6.75 / 1.0 * 14.0 / 16.0; // Swapped a gear, slightly different ratio... grr...
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

    public static class MAVCoderConstants {
        public static final double MAV_2_MIN_OUTPUT = 0.03;
        public static final double MAV_2_MAX_OUTPUT = 0.97;
        public static final double MAV_2_POSITION_FACTOR = 360.0 / (MAV_2_MAX_OUTPUT - MAV_2_MIN_OUTPUT);
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

    // Limelight 3 Position: (Forward: 0.262m, Up (From ground): 0.183m, L/R: 0.0m, Roll: 0.0deg, Pitch: 35.0deg, Yaw: 0.0deg)
    // Limelight 2+ Position: (Forward: -0.290m, Up (From ground): 0.345m, L/R: 0.014m, Roll: 0.0deg, Pitch: 0.0deg, Yaw: 180.0deg)
    public static enum ArmElevatorSetpoints implements ArmElevatorSetpoint {
        ZERO(new Rotation2d(0), 0),
        HOME(Rotation2d.fromDegrees(35.0), Meters.convertFrom(0.0, Inches)),
        TEST(Rotation2d.fromDegrees(45.0), Meters.convertFrom(12.0, Inches));

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
        public static final double ARM_PID_P = 0.75;
        public static final double ARM_PID_I = 0.0001;
        public static final double ARM_PID_D = 0.0;
        public static final double ARM_PID_F = 0.35;

        public static final double ARM_LENGTH = Meters.convertFrom(16.0, Inches);

        public static final Rotation2d ARM_ANGLE_DEADZONE = Rotation2d.fromDegrees(5.0);

        public static final double ARM_RATIO = Rotation2d.fromDegrees(7.872).getRadians();

        public static final Rotation2d MAX_ARM_ANGLE = Rotation2d.fromDegrees(50.0);
        public static final Rotation2d MIN_ARM_ANGLE = Rotation2d.fromDegrees(-20.0);

        public static final Rotation2d MAX_SAFE_ANGLE = Rotation2d.fromRotations(45.0);
    }

    public static class ShooterConstants {
        public static final double MAX_EMPTY_LIDAR_DISTANCE = Meters.convertFrom(10.0, Millimeters);
    }

    public static class ElevatorConstants {
        public static final double ELEVATOR_PID_P = 4.0;
        public static final double ELEVATOR_PID_I = 0.0;
        public static final double ELEVATOR_PID_D = 0.0;
        public static final double ELEVATOR_PID_F = 2.0;

        public static final double ELEVATOR_HEIGHT_DEADZONE = Meters.convertFrom(2.0, Centimeters);

        public static final double BLOCKING_HEIGHT = Meters.convertFrom(3.0, Inches);

        public static final Translation3d ELEVATOR_TRANSLATION = new Translation3d(0, Meters.convertFrom(2.0, Inches), Meters.convertFrom(4.0, Inches));

        public static final double ELEVATOR_RATIO = Meters.convertFrom(18.0, Millimeters) / 1.0; //Find empericaly

        public static final double MAX_ELEVATOR_HEIGHT = /*Meters.convertFrom(40.0, Inches)*/ 0.5;
        public static final double MIN_ELEVATOR_HEIGHT = Meters.convertFrom(0, Inches);
    }
}
