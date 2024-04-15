package frc.robot.commands;

import static frc.robot.constants.UniversalConstants.getGlobalPositions;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.TargetPositionState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.AngleStoppedFacePositionState;

public class AngleStoppedAutoAimCommand extends ParallelRaceGroup {
    public AngleStoppedAutoAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        super(
            new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), getGlobalPositions().SPEAKER_TARGET_POSITION) {
                @Override
                public void end(boolean interrupted) { }
            },
            new AngleStoppedFacePositionState(drivetrain, xSpeed, ySpeed, getGlobalPositions().SPEAKER_POSITION)
        );
    }
}
