package frc.robot.subsystems.drivetrain.drivers;

import java.util.function.DoubleSupplier;

import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;

public class ControllerDrivers {
    public static class XDriver implements Driver {
        private DoubleSupplier xSpeed;

        public XDriver(DoubleSupplier xSpeed) {
            this.xSpeed = xSpeed;
        }

        @Override
        public double getSpeed() {
            return -xSpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
        }

        @Override
        public double getPosition() {
            return 0.0;
        }
    }

    public static class YDriver implements Driver {
        private DoubleSupplier ySpeed;

        public YDriver(DoubleSupplier ySpeed) {
            this.ySpeed = ySpeed;
        }

        @Override
        public double getSpeed() {
            return ySpeed.getAsDouble() * MAX_TRANSLATIONAL_SPEED;
        }

        @Override
        public double getPosition() {
            return 0.0;
        }
    }

    public static class ThetaDriver implements Driver {
        private DoubleSupplier thetaSpeed;

        public ThetaDriver(DoubleSupplier thetaSpeed) {
            this.thetaSpeed = thetaSpeed;
        }

        @Override
        public double getSpeed() {
            return thetaSpeed.getAsDouble() * MAX_ROTATIONAL_SPEED;
        }

        @Override
        public double getPosition() {
            return 0.0;
        }
    }
}