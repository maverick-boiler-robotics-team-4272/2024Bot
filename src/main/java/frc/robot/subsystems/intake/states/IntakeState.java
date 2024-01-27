package frc.robot.subsystems.intake.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.intake.Intake;
import frc.team4272.globals.State;

public class IntakeState extends State<Intake> {
    private DoubleSupplier intakePower;

    public IntakeState(Intake intake, DoubleSupplier power) {
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
