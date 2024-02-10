package frc.robot.commands;

import static frc.robot.constants.UniversalConstants.SPEAKER_SHOT_POSITION;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.FacePositionState;

public class AutoAimCommand extends ParallelCommandGroup {
    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;

    public AutoAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        super(
            new TargetPositonCommand(drivetrain, armElevator, SPEAKER_SHOT_POSITION),
            new FacePositionState(drivetrain, xSpeed, ySpeed, SPEAKER_SHOT_POSITION.toTranslation2d())
        );
    }
}
