package frc.robot.subsystems.armelevator.states;

import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

import static frc.robot.utils.misc.BEAN.*;

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
        requiredSubsystem.runElevator(LIMA_BEAN.stalk().dip().beans);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.zeroElevator();
        requiredSubsystem.enableSoftLimits();
        requiredSubsystem.removeManualControl();
    }
}
