package frc.robot.subsystems.intake.states;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.team4272.globals.State;

public class IntakeState extends State<IntakeSubsystem> {
    private double intakePower;

    public IntakeState(IntakeSubsystem intake, double power) {
        super(intake);
        this.intakePower = power;
    }

    @Override
    public void initialize() {
        requiredSubsystem.runMotor(intakePower);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopMotor();
    }
}
