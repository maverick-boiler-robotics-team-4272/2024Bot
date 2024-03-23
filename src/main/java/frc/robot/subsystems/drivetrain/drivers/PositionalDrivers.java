package frc.robot.subsystems.drivetrain.drivers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionalDrivers {
    private static final double POSITION_PID_P = 2.0;
    private static final double POSITION_PID_I = 0.0;
    private static final double POSITION_PID_D = 0.0;
    
    private static final double ROTATION_PID_P = 2.1;
    private static final double ROTATION_PID_I = 0.015;
    private static final double ROTATION_PID_D = 0.0;

    private static final PIDController X_CONTROLLER = new PIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D);
    private static final PIDController Y_CONTROLLER = new PIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D);
    private static final PIDController THETA_CONTROLLER = new PIDController(ROTATION_PID_P, ROTATION_PID_I, ROTATION_PID_D);

    public static class XDriver implements Driver {
        private Drivetrain drivetrain;
        private PIDController controller;
        private double desiredXPosition;

        public XDriver(Drivetrain drivetrain, PIDController controller) {
            this.drivetrain = drivetrain;
            this.controller = controller;
        }

        public XDriver(Drivetrain drivetrain) {
            this(drivetrain, X_CONTROLLER, null);
        }
        
        public void setDesiredXPosition(double xPosition) {
            this.desiredXPosition = xPosition;
        }

        @Override
        public double getSpeed() {
            Pose2d desiredPose = drivetrain.getRobotPose();

            return -controller.calculate(desiredPose.getX(), desiredXPosition);
        }


        @Override
        public double getPosition() {
            return desiredXPosition;
        }

    }

    public static class YDriver implements Driver {
        private Drivetrain drivetrain;
        private PIDController controller;
        private double desiredYPosition;

        public YDriver(Drivetrain drivetrain, PIDController controller) {
            this.drivetrain = drivetrain;
            this.controller = controller;
        }

        public YDriver(Drivetrain drivetrain) {
            this(drivetrain, Y_CONTROLLER);
        }

        public void setDesiredYPosition(double yPosition) {
            this.desiredYPosition = yPosition;
        }

        @Override
        public double getSpeed() {
            Pose2d desiredPose = drivetrain.getRobotPose();

            return controller.calculate(desiredPose.getY(), desiredYPosition);
        }

        @Override
        public double getPosition() {
            return desiredYPosition;
        }

    }

    public static class ThetaDriver implements Driver {
        private Drivetrain drivetrain;
        private PIDController controller;
        private Rotation2d desiredAngle;

        public ThetaDriver(Drivetrain drivetrain, PIDController controller) {
            this.drivetrain = drivetrain;
            this.controller = controller;

            if(!this.controller.isContinuousInputEnabled())
               this.controller.enableContinuousInput(-Math.PI, Math.PI); 
        }

        public ThetaDriver(Drivetrain drivetrain) {
            this(drivetrain, THETA_CONTROLLER);
        }

        public void setDesiredAngle(Rotation2d rotation) {
            this.desiredAngle = rotation;
        }

        @Override
        public double getSpeed() {
            Pose2d desiredPose = drivetrain.getRobotPose();

            return -controller.calculate(desiredPose.getRotation().getRadians(), desiredAngle.getRadians());
        }

        @Override
        public double getPosition() {
            return desiredAngle.getRadians();
        }
    }
}