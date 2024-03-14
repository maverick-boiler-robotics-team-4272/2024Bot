package frc.robot.commands.autos;

// Commands / States
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.armelevator.states.*;
import frc.robot.subsystems.shooter.states.*;

// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

// Constants
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;

public class FireAndSit extends SequentialCommandGroup {
    public FireAndSit(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new ParallelCommandGroup(
                new GoToArmElevatorState(armElevator, SUB_SHOT),
                new WaitCommand(0.75).andThen(
                    new AutoShootState(shooter, 0.9, 0.5)
                )
            ),
            new GoToArmElevatorState(armElevator, HOME),
            new InstantCommand(drivetrain::resetModules)
            // new DriveState(drivetrain, () -> 0.5, () -> 0.2, () -> 0.0, false)
        );
    }
}
