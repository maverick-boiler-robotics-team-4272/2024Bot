package frc.robot.subsystems.drivetrain.drivers;

import edu.wpi.first.math.controller.PIDController;

public class PathDrivers {
    public static class XDriver extends PositionalDrivers.XDriver {
        private double feedForward;

        public XDriver(Drivetrain drivetrain, PIDController controller) {
            super(drivetrain, controller);
        }

        public XDriver(Drivetrain drivetrain) {
            super(drivetrain);
        }

        public void setFeedForward(double feedForward) {
            this.feedForward = feedForward;
        }

        @Override
        public double getSpeed() {
            return super.getSpeed() - feedForward;
        }
    }

    public static class YDriver extends PositionalDrivers.YDriver {
        private double feedForward;

        public YDriver(Drivetrain drivetrain, PIDController controller) {
            super(drivetrain, controller);
        }

        public YDriver(Drivetrain drivetrain) {
            super(drivetrain);
        }

        public void setFeedForward(double feedForward) {
            this.feedForward = feedForward;
        }

        @Override
        public double getSpeed() {
            return super.getSpeed() + feedForward;
        }
    }

    public static class ThetaDriver extends PositionalDrivers.ThetaDriver {
        private double feedForward;

        public ThetaDriver(Drivetrain drivetrain, PIDController controller) {
            super(drivetrain, controller);
        }

        public ThetaDriver(Drivetrain drivetrain) {
            super(drivetrain);
        }

        public void setFeedForward(double feedForward) {
            this.feedForward = feedForward;
        }

        @Override
        public double getSpeed() {
            return super.getSpeed() - feedForward;
        }
    }
}