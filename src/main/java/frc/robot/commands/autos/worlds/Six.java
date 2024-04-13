package frc.robot.commands.autos.worlds;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.START_LINE;
import static frc.robot.constants.UniversalConstants.getGlobalPositions;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.FacePositionState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;
import frc.robot.subsystems.shooter.states.ShootState;

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
