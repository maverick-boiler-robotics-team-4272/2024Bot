package frc.robot.commands.autos.worlds;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;

public class Six extends BaseAuto {
    public Six(Drivetrain drivetrain, ArmElevatorSubsystem armelevator, Shooter shooter) {
        super(
            drivetrain, 
            true, 
            getGlobalTrajectories().P_6
        );

        addCommands(
            pathFollowCommand,
            new AutoShootState(shooter, 1.0, 1.0)
        );
    }
}
