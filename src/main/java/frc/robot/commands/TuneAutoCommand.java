// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Commands / States
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.states.PathFollowState;

// Subsystems
import frc.robot.subsystems.drivetrain.Drivetrain;

// Constants
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

public class TuneAutoCommand extends SequentialCommandGroup {
    public TuneAutoCommand(Drivetrain drivetrain) {
        super(
            new PathFollowState(drivetrain, getGlobalTrajectories().TUNE_PATH, false, false)
        );
    }
}
