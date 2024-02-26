package frc.robot.subsystems.drivetrain.states;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.team4272.globals.State;

public class ResetToLimelightState extends State<Drivetrain> {
    private Limelight limelight;

    public ResetToLimelightState(Drivetrain drivetrain, Limelight limelight) {
        super(drivetrain);

        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        Pose2d limelightPose = limelight.getRobotPose();
        requiredSubsystem.setRobotPose(new Pose2d(limelightPose.getTranslation(), limelightPose.getRotation().unaryMinus()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
