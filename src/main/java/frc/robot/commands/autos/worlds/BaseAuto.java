package frc.robot.commands.autos.worlds;

import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.START_LINE;
import static frc.robot.constants.UniversalConstants.getGlobalPositions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.utils.paths.Path;

public class BaseAuto extends SequentialCommandGroup {
    protected final PathFollowWithEvents pathFollowCommand;
    protected final ParallelDeadlineGroup startShot;
    protected final ParallelRaceGroup endShot;

    public BaseAuto(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter, boolean disable, Path path) {
        super(
            new InstantCommand(() -> {
                drivetrain.resetModules();
                if(disable)
                    drivetrain.disableVisionFusion();
            })
        );

        pathFollowCommand = new PathFollowWithEvents(
            new PathFollowState(drivetrain, path, true, false),
            path
        );

        startShot = new ParallelDeadlineGroup(
            new ParallelCommandGroup(
                new GoToArmElevatorState(armElevator, START_LINE),
                new AutoShootState(shooter, 1.0, 1.0, 0.6)
            ),
            new FacePositionState(drivetrain, () -> 0, () -> 0, getGlobalPositions().SPEAKER_TARGET_POSITION.toTranslation2d())
        );

        endShot = new ParallelRaceGroup(
            new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
            new AutoShootState(shooter, 1.0, 1.0).beforeStarting(new WaitCommand(0.1))
        );
    }
}
