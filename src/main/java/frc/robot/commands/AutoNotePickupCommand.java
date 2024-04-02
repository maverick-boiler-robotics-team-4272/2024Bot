package frc.robot.commands;

import static frc.robot.constants.RobotConstants.LimelightConstants.*;
import static frc.robot.constants.TelemetryConstants.Limelights.*;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.GoToPositionState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

import static frc.robot.utils.misc.BEAN.*;

public class AutoNotePickupCommand extends SequentialCommandGroup {
    public AutoNotePickupCommand(Drivetrain drivetrain, IntakeSubsystem intake, Shooter shooter) {
        addCommands(
            Commands.defer(
            () -> {
                Pose2d notePose = drivetrain.getNotePose();
                Pose2d ePose = drivetrain.getRobotPose();

                return new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        drivetrain.disableVisionFusion();
                    }, drivetrain),
                    new ParallelDeadlineGroup(
                        new IntakeFeedCommand(intake, shooter, BAKED_BEAN.beans),
                        new GoToPositionState(drivetrain, new Pose2d(notePose.getX(), notePose.getY(), ePose.getTranslation().minus(notePose.getTranslation()).getAngle()))
                    ),
                    new GoToPositionState(drivetrain, ePose),
                    new InstantCommand(() -> {
                        drivetrain.enableVisionFusion();
                    }, drivetrain)
                );
            }, Set.of(drivetrain, intake, shooter))
        );
    }
}
