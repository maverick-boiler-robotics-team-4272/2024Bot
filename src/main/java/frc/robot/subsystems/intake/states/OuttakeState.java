package frc.robot.subsystems.intake.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.team4272.globals.State;

public class OuttakeState extends State<IntakeSubsystem> {
    private DoubleSupplier outtakePower;

    public OuttakeState(IntakeSubsystem intake, DoubleSupplier power) {
        super(intake);
        this.outtakePower = power;
    }

    @Override
    public void execute() {
        requiredSubsystem.runMotor(-outtakePower.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopMotor();
    }
}