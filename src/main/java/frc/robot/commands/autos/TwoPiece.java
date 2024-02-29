package frc.robot.commands.autos;

import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.HOME;
import static frc.robot.constants.TelemetryConstants.Limelights.FRONT_LIMELIGHT;
import static frc.robot.constants.UniversalConstants.getGlobalPositions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.IntakeFeedCommand;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
import frc.robot.subsystems.drivetrain.states.RotLockState;
import frc.robot.subsystems.drivetrain.states.SetHeadingState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;
import frc.robot.subsystems.shooter.states.LidarStoppedFeedState;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter, IntakeSubsystem intake) {
        super(
            new InstantCommand(drivetrain::resetModules),
            new GoToArmElevatorState(armElevator, HOME),
            new ResetHeadingState(drivetrain),
            new ParallelDeadlineGroup(
                new Command() {
                    @Override
                    public boolean isFinished() {
                        return FRONT_LIMELIGHT.isValidTarget();
                    }
                },
                new DriveState(drivetrain, () -> 0, () -> 0, () -> 0.5)
            ),
            new RotLockState(drivetrain, () -> 0, () -> 0.1, () -> getGlobalPositions().TO_DRIVERSTATION).withTimeout(3.0),
            new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0).raceWith(
                new AutoShootState(shooter, 1, 1).beforeStarting(
                    new WaitCommand(0.5)
                )
            ),
            new RotLockState(drivetrain, () -> 0, () -> 0.0, () -> getGlobalPositions().TO_DRIVERSTATION).withTimeout(0.5),
            new RotLockState(drivetrain, () -> 0, () -> 0.5, () -> getGlobalPositions().TO_DRIVERSTATION).alongWith(
                new IntakeFeedCommand(intake, shooter, () -> 1.0)
            ).withTimeout(0.75),
            new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0).raceWith(
                new AutoShootState(shooter, 1, 1).beforeStarting(
                    new LidarStoppedFeedState(shooter, () -> 0.5)
                )
            ),
            new RotLockState(drivetrain, () -> 0, () -> 0, () -> getGlobalPositions().TO_DRIVERSTATION),
            new RotLockState(drivetrain, () -> 0, () -> 0, () -> getGlobalPositions().TO_DRIVERSTATION.plus(Rotation2d.fromDegrees(180)))
        );
    }
}
