package frc.robot.subsystems.intake.states;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.team4272.globals.State;

public class OuttakeState extends State<IntakeSubsystem> {
    private double outtakePower;

    public OuttakeState(IntakeSubsystem intake, double power) {
        super(intake);
        this.outtakePower = power;
    }

    @Override
    public void initialize() {
        requiredSubsystem.runMotor(-outtakePower);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopMotor();
    }
}