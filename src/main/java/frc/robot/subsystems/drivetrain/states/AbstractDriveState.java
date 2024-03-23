package frc.robot.subsystems.drivetrain.states;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.team4272.globals.State;

import frc.robot.subsystems.drivetrain.drivers.Driver;

public abstract class AbstractDriveState<X extends Driver, Y extends Driver, T extends Driver> extends State<Drivetrain> {
    protected final X xDriver;
    protected final Y yDriver;
    protected final T thetaDriver;
    
    public AbstractDriveState(Drivetrain drivetrain, X xDriver, Y yDriver, T thetaDriver) {
        super(drivetrain);

        this.xDriver = xDriver;
        this.yDriver = yDriver;
        this.thetaDriver = thetaDriver;
    }

    public abstract boolean isFieldRelative();

    @Override
    public void execute() {
        drive();
    }

    protected void drive() {
        if(isFieldRelative()) {
            requiredSubsystem.driveFieldOriented(xDriver.getSpeed(), yDriver.getSpeed(), thetaDriver.getSpeed());
        } else {
            requiredSubsystem.drive(xDriver.getSpeed(), yDriver.getSpeed(), thetaDriver.getSpeed());
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.drive(0, 0, 0);
    }
}
