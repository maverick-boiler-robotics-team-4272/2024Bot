package frc.robot.subsystems.armelevator.states;

import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

public class ZeroElevatorState extends State<ArmElevatorSubsystem> {
    public ZeroElevatorState(ArmElevatorSubsystem armElevator) {
        super(armElevator);
    }

    @Override
    public void initialize() {
        requiredSubsystem.disableSoftLimits();
    }

    @Override
    public void execute() {
        requiredSubsystem.runElevator(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.zeroElevator();
        requiredSubsystem.enableSoftLimits();
        requiredSubsystem.removeManualControl();
    }
}
