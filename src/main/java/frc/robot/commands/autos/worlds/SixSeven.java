package frc.robot.commands.autos.worlds;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class SixSeven extends BaseAuto {
    public SixSeven(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(drivetrain, armElevator, shooter, true, getGlobalTrajectories().P_67);

        addCommands(
            startShot,
            pathFollowCommand
        );
    }
}
