package frc.robot.subsystems.armelevator.states;

import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

import static frc.robot.utils.misc.BEAN.*;

public class LatchState extends State<ArmElevatorSubsystem> {
    public LatchState(ArmElevatorSubsystem armElevator) {
        super(armElevator);
    }

    @Override
    public void initialize() {
        
        requiredSubsystem.latch();
        requiredSubsystem.runElevator(LIMA_BEAN.stalk().beans);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.unlatch();
        requiredSubsystem.removeManualControl();
    }
}
