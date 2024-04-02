package frc.robot.subsystems.armelevator.states;

import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

public class LatchState extends State<ArmElevatorSubsystem> {
    public LatchState(ArmElevatorSubsystem armElevator) {
        super(armElevator);
    }

    @Override
    public void initialize() {
        
        requiredSubsystem.latch();
        requiredSubsystem.runElevator(-0.1);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.unlatch();
        requiredSubsystem.removeManualControl();
    }
}