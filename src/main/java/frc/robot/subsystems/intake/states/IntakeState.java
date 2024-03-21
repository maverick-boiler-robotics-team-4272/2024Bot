package frc.robot.subsystems.intake.states;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.team4272.globals.State;

public class IntakeState extends State<IntakeSubsystem> {
    private double intakePower;

    private int outtakeTime;
    private boolean needsUpdate;

    public IntakeState(IntakeSubsystem intake, double power) {
        super(intake);
        this.intakePower = power;
    }

    @Override
    public void initialize() {
        outtakeTime = -1;
        needsUpdate = true;
    }

    @Override
    public void execute() {
        // If outtake time is not the "Dont outtake" number
        if(outtakeTime != -1) {
            // If we need to change the power in the motor
            if(needsUpdate) {
                requiredSubsystem.runMotor(-intakePower);
                needsUpdate = false;
            }

            // If we have been outtaking for half a second
            if(outtakeTime >= 25) {
                outtakeTime = -1;
                needsUpdate = true;
            }

            outtakeTime++;
        } else {
            // If we need to change the power in the motor
            if(needsUpdate) {
                requiredSubsystem.runMotor(intakePower);
                needsUpdate = false;
            }

            // If the motor is stalling, tell it to outtake
            if(requiredSubsystem.isMotorStalling()) {
                outtakeTime = 0;
                needsUpdate = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopMotor();
    }
}
