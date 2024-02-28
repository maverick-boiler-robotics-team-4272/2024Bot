package frc.robot.commands;

import static frc.robot.constants.UniversalConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.TargetPositionState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.FacePositionState;

public class AutoAimCommand extends ParallelCommandGroup {
    public AutoAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        super(
            new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), getGlobalPositions().SPEAKER_SHOT_POSITION),
            new FacePositionState(drivetrain, xSpeed, ySpeed, getGlobalPositions().SPEAKER_SHOT_POSITION.toTranslation2d())
        );
    }
}
