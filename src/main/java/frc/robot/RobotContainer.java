// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.HomemadeAuto;
import frc.robot.commands.StressTestAuto;
import frc.robot.commands.TestAutoCommand;
import frc.robot.commands.TuneAutoCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.team4272.controllers.XboxController;
import frc.team4272.controllers.utilities.JoystickAxes;
import frc.team4272.controllers.utilities.JoystickAxes.DeadzoneMode;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.FacePositionState;
import frc.robot.subsystems.drivetrain.states.GoToPositionState;
import frc.robot.subsystems.drivetrain.states.PositionState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
import frc.robot.subsystems.drivetrain.states.ResetToLimelightState;
import static frc.robot.constants.AutoConstants.Paths.*;
import static frc.robot.constants.TelemetryConstants.Limelights.CENTER_LIMELIGHT;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;
import static frc.robot.constants.UniversalConstants.AMP_POSE;
import static frc.robot.constants.UniversalConstants.SPEAKER_POSITION;

import com.pathplanner.lib.auto.NamedCommands;
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
    Intake intake = new Intake();

    // The robots IO devices are defined here
    XboxController driveController = new XboxController(0);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureAutoChoosers();
        registerNamedCommands();
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
            new ResetToLimelightState(drivetrain, CENTER_LIMELIGHT)
        );

        new Trigger(driveController.getTrigger("left")::isTriggered).whileTrue(
            new IntakeState(intake, () -> driveController.getTrigger("left").getValue() * -0.6)
        );

        new Trigger(driveController.getTrigger("right")::isTriggered).whileTrue(
            new IntakeState(intake, () -> driveController.getTrigger("right").getValue() * 0.6)
        );

        new Trigger(driveController.getButton("a")::get).whileTrue(
            new FacePositionState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, SPEAKER_POSITION)
        );

        new Trigger(driveController.getButton("x")::get).whileTrue(
            new PositionState(drivetrain, () -> new Pose2d(Units.inchesToMeters(72.5), Units.inchesToMeters(300), new Rotation2d(Math.PI / 2)))
        );

        new Trigger(driveController.getButton("rightBumper")::get).whileTrue(
            new GoToPositionState(drivetrain, AMP_POSE)
        );
    }

    public void configureAutoChoosers() {
        CONTAINER_CHOOSER.setDefaultOption("Red", RED_TRAJECTORIES);
        CONTAINER_CHOOSER.addOption("Blue", BLUE_TRAJECTORIES);

        AUTO_CHOOSER.setDefaultOption("Test Path", () -> new TestAutoCommand(drivetrain));
        AUTO_CHOOSER.addOption("Tune Path", () -> new TuneAutoCommand(drivetrain).repeatedly());
        AUTO_CHOOSER.addOption("Stress Path", () -> new StressTestAuto(drivetrain));
        AUTO_CHOOSER.addOption("HomeMade TBD", () -> new HomemadeAuto(drivetrain));
        

        AUTO_TABLE.putData("Auto Chooser", AUTO_CHOOSER);
        AUTO_TABLE.putData("Side Chooser", CONTAINER_CHOOSER);
    }

    public void registerNamedCommands() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if(!hasGlobalTrajectories()) {
            setGlobalTrajectories(CONTAINER_CHOOSER.getSelected());
        }

        return AUTO_CHOOSER.getSelected().get();
    }
}
