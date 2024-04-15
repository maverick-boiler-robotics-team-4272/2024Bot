package frc.robot.commands.autos.worlds;

import static frc.robot.constants.UniversalConstants.getGlobalPositions;
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.START_LINE;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.FacePositionState;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;

public class SixFive extends SequentialCommandGroup {
    public SixFive(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new GoToArmElevatorState(armElevator, START_LINE),
                    new AutoShootState(shooter, 1.0, 1.0, 0.6)
                ),
                new FacePositionState(drivetrain, () -> 0, () -> 0, getGlobalPositions().SPEAKER_TARGET_POSITION.toTranslation2d(
            ),
            new PathFollowWithEvents(new PathFollowState(drivetrain, getGlobalTrajectories().P_65, true, false), getGlobalTrajectories().P_65),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new AutoShootState(shooter, 1.0, 1.0).beforeStarting(new WaitCommand(0.1))
            )
        );
    }
}
