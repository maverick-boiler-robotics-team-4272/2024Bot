package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ArmElevatorSetpoints;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TargetPositonCommand extends Command {
    Drivetrain drivetrain;
    ArmElevatorSubsystem armElevator;
    Translation3d target;

    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;

    public TargetPositonCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Translation3d target, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        this.drivetrain = drivetrain;
        this.armElevator = armElevator;
        this.target = target;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;

        addRequirements(drivetrain, armElevator);
    }

    @Override
    public void execute() {
        armElevator.targetPos(drivetrain.getRobotPose().getTranslation(), target);
    }

    @Override
    public void end(boolean interrupted) {
        armElevator.goToPos(ArmElevatorSetpoints.HOME);
    }
}
