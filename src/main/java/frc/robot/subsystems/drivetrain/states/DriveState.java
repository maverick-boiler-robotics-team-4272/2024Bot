package frc.robot.subsystems.drivetrain.states;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.team4272.globals.State;

public class DriveState extends State<Drivetrain> {
    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;
    private DoubleSupplier thetaSpeed;

    public DriveState(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier thetaSpeed) {
        super(drivetrain);

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.thetaSpeed = thetaSpeed;
    }

    @Override
    public void execute() {
        requiredSubsystem.driveFieldOriented(ySpeed.getAsDouble(), -xSpeed.getAsDouble(), thetaSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.drive(0, 0, 0);
    }
}
