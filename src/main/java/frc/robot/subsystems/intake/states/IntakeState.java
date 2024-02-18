package frc.robot.subsystems.intake.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.team4272.globals.State;

public class IntakeState extends State<IntakeSubsystem> {
    private DoubleSupplier intakePower;

    public IntakeState(IntakeSubsystem intake, DoubleSupplier power) {
        super(intake);
        this.intakePower = power;
    }

    @Override
    public void execute() {
        requiredSubsystem.runMotor(intakePower.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopMotor();
    }
}
