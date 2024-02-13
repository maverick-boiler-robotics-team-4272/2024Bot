package frc.robot.subsystems.armelevator.states;

import frc.robot.constants.RobotConstants.ArmElevatorSetpoint;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

public class GoToArmElevatorState extends State<ArmElevatorSubsystem> {
    ArmElevatorSetpoint setpoint;

    public GoToArmElevatorState(ArmElevatorSubsystem armElevator, ArmElevatorSetpoint setpoint) {
        super(armElevator);

        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        requiredSubsystem.goToPos(setpoint);
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.isAtPosition();
    }
}
