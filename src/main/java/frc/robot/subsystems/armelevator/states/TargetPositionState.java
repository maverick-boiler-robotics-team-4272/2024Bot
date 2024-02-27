package frc.robot.subsystems.armelevator.states;


import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.HOME;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.RobotConstants.ArmElevatorSetpoints;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

public class TargetPositionState extends State<ArmElevatorSubsystem> {
    Supplier<Translation2d> drivetrainPos;
    Translation3d target;
    BooleanSupplier lidar;

    public TargetPositionState(ArmElevatorSubsystem armElevator, Supplier<Translation2d> driveTrainPos, Translation3d target, BooleanSupplier lidar) {
        super(armElevator);
        this.target = target;
        this.drivetrainPos = driveTrainPos;
        this.lidar = lidar;
    }

    public TargetPositionState(ArmElevatorSubsystem armElevator, Supplier<Translation2d> driveTrainPos, Translation3d target) {
        this(armElevator, driveTrainPos, target, () -> true);
    }

    @Override
    public void execute() {
        if(lidar.getAsBoolean()) {
            requiredSubsystem.targetPos(drivetrainPos.get(), target);
        } else {
            requiredSubsystem.goToPos(HOME);
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.goToPos(HOME);
    }
}
