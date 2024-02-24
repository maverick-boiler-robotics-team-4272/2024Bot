package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathFollowWithAimCommand;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TwoCenterRush extends SequentialCommandGroup {
    public TwoCenterRush(Drivetrain drivetrain, ArmElevatorSubsystem armElevator) {
        super(
            new PathFollowWithAimCommand(drivetrain, armElevator, getGlobalTrajectories().TWO_CENTER_RUSH)
        );
    }
}
