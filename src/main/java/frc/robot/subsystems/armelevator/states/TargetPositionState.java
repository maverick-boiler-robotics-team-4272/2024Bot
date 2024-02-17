package frc.robot.subsystems.armelevator.states;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.RobotConstants.ArmElevatorSetpoints;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

public class TargetPositionState extends State<ArmElevatorSubsystem> {
    Supplier<Translation2d> drivetrainPos;
    Translation3d target;

    public TargetPositionState(ArmElevatorSubsystem armElevator, Supplier<Translation2d> driveTrainPos, Translation3d target) {
        super(armElevator);
        this.target = target;
        this.drivetrainPos = driveTrainPos;
    }

    @Override
    public void execute() {
        requiredSubsystem.targetPos(drivetrainPos.get(), target);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.goToPos(ArmElevatorSetpoints.HOME);
    }
}
