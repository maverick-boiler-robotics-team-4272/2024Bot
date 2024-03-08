package frc.robot.commands.autos;


// Commands / States
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.armelevator.states.*;
import frc.robot.subsystems.drivetrain.states.*;
import frc.robot.subsystems.shooter.states.*;

// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

// Math
import edu.wpi.first.math.geometry.Rotation2d;

// Constants
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;
import static frc.robot.constants.TelemetryConstants.Limelights.*;
import static frc.robot.constants.UniversalConstants.getGlobalPositions;

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
                new IntakeFeedCommand(intake, shooter, 1.0)
            ).withTimeout(0.75),
            new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0).raceWith(
                new AutoShootState(shooter, 1, 1).beforeStarting(
                    new LidarStoppedFeedState(shooter,  0.5)
                )
            ),
            new RotLockState(drivetrain, () -> 0, () -> 0, () -> getGlobalPositions().TO_DRIVERSTATION),
            new RotLockState(drivetrain, () -> 0, () -> 0, () -> getGlobalPositions().TO_DRIVERSTATION.plus(Rotation2d.fromDegrees(180)))
        );
    }
}
