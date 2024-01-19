// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.team4272.controllers.XboxController;
import frc.team4272.controllers.utilities.JoystickAxes;
import frc.team4272.controllers.utilities.JoystickAxes.DeadzoneMode;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
// import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.intake.states.IntakeState;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    Drivetrain drivetrain = new Drivetrain();
    // IntakeSubsystem intake = new IntakeSubsystem();

    // The robots IO devices are defined here
    XboxController driveController = new XboxController(0);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        JoystickAxes driveLeftAxes = driveController.getAxes("left");
        JoystickAxes driveRightAxes = driveController.getAxes("right");
        driveLeftAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kMagnitude).setPowerScale(3);
        driveRightAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kXAxis).setPowerScale(2.5);
        
        drivetrain.setDefaultCommand(
            new DriveState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, driveRightAxes::getDeadzonedX)
        );

        new Trigger(driveController.getButton("b")::get).onTrue(
            new ResetHeadingState(drivetrain)
        );

        new Trigger(driveController.getButton("y")::get).onTrue(
            new InstantCommand(drivetrain::setToZero, drivetrain)
        );

        // new Trigger(() -> driveController.getTrigger("left").getValue() != 0.0).whileTrue(
        //     new IntakeState(intake, () -> driveController.getTrigger("left").getValue() * -0.6)
        // );

        // new Trigger(() -> driveController.getTrigger("right").getValue() != 0.0).whileTrue(
        //     new IntakeState(intake, () -> driveController.getTrigger("right").getValue() * 0.6)
        // );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
