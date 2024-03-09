package frc.robot.utils.misc;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Pausable {
    public void pause();
    public void unpause();

    // Command Methods. These are all I need
    public Set<Subsystem> getRequirements();

    default public void initialize() {

    }

    default public void execute() {

    }

    default public void end(boolean interrupted) {

    }

    default public boolean isFinished() {
        return false;
    }
}
