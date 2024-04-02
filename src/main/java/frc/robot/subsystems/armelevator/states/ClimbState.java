package frc.robot.subsystems.armelevator.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.team4272.globals.State;

public class ClimbState extends State<ArmElevatorSubsystem> {
    DoubleSupplier speed;

    public ClimbState(ArmElevatorSubsystem armElevator, DoubleSupplier speed) {
        super(armElevator);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        requiredSubsystem.elevatorGoNyrooom();
        requiredSubsystem.goToPos(Rotation2d.fromDegrees(0), 0);
    }

    @Override
    public void execute() {
        requiredSubsystem.runElevator(-speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.runElevator(0);
        requiredSubsystem.removeManualControl();
    }
}
