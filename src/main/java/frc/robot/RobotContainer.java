// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.HomemadeAuto;
import frc.robot.commands.StressTestAuto;
import frc.robot.commands.TestAutoCommand;
import frc.robot.commands.TuneAutoCommand;
import frc.robot.constants.Norms;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.FeedState;
import frc.robot.subsystems.shooter.states.ShootState;
import frc.team4272.controllers.XboxController;
import frc.team4272.controllers.utilities.JoystickAxes;
import frc.team4272.controllers.utilities.JoystickAxes.DeadzoneMode;
import frc.team4272.controllers.utilities.JoystickPOV.Direction;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.FacePositionState;
import frc.robot.subsystems.drivetrain.states.GoToPositionState;
import frc.robot.subsystems.drivetrain.states.PathFindToPositionState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
import frc.robot.subsystems.drivetrain.states.ResetToLimelightState;
import static frc.robot.constants.AutoConstants.Paths.*;
import static frc.robot.constants.TelemetryConstants.Limelights.CENTER_LIMELIGHT;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;
import static frc.robot.constants.UniversalConstants.AMP_POSE;
import static frc.robot.constants.UniversalConstants.SPEAKER_POSITION;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;

import java.util.Map;
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
    IntakeSubsystem intake = new IntakeSubsystem();
    ArmElevatorSubsystem armElevator = new ArmElevatorSubsystem();
    Shooter shooter = new Shooter();

    int driverDPadValue = -1;

    // The robots IO devices are defined here
    XboxController driveController = new XboxController(0);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Norms.initialize(drivetrain);

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
        configureDriverBindings();
        configureOperatorBindings();
    }

    public void configureDriverBindings() {
        JoystickAxes driveLeftAxes = driveController.getAxes("left");
        driveLeftAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kMagnitude).setPowerScale(3);

        JoystickAxes driveRightAxes = driveController.getAxes("right");
        driveRightAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kXAxis).setPowerScale(2.5);
        
        armElevator.setDefaultCommand(new GoToArmElevatorState(armElevator, HOME));

        drivetrain.setDefaultCommand(
            new DriveState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, driveRightAxes::getDeadzonedX)
        );
        
        new Trigger(driveController.getTrigger("left")::isTriggered).whileTrue(
            // new IntakeState(intake, () -> -driveController.getTrigger("left").getValue())
            new ParallelCommandGroup(
                new GoToArmElevatorState(armElevator, HOME),
                new IntakeState(intake, () -> driveController.getTrigger("left").getValue()),
                new FeedState(shooter, () -> driveController.getTrigger("left").getValue())
            )
            // new IntakeState(intake, () -> -1.0)
        );
        
        new Trigger(driveController.getTrigger("right")::isTriggered).whileTrue(
            new ParallelCommandGroup(
                new IntakeState(intake, () -> -driveController.getTrigger("right").getValue()),
                new FeedState(shooter, () -> -driveController.getTrigger("right").getValue())
            )
        );

        new Trigger(driveController.getButton("leftBumper")::get).whileTrue(
            new ShootState(shooter, () -> 1.0, driveController.getButton("rightBumper")::get)
        );

        new Trigger(driveController.getButton("rightBumper")::get).whileTrue(
            new SelectCommand<Integer>(Map.of(
                0,
                new ParallelCommandGroup(
                    new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 1.0)),
                    new PathFindToPositionState(drivetrain, AMP_POSE)
                ),
                90, 
                new GoToPositionState(drivetrain, AMP_POSE)
                ),
                () -> driverDPadValue
            )
        ).onFalse(
            new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 0.0))
        );
        
        new Trigger(driveController.getButton("b")::get).onTrue(
            new ResetHeadingState(drivetrain)
        );

        new Trigger(driveController.getButton("y")::get).onTrue(
            new ResetToLimelightState(drivetrain, CENTER_LIMELIGHT)
        );

        new Trigger(driveController.getButton("a")::get).whileTrue(
            new FacePositionState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, SPEAKER_POSITION)
        );

        new Trigger(driveController.getButton("x")::get).whileTrue(
            new GoToArmElevatorState(armElevator, TEST).repeatedly()
        );

        new Trigger(() -> !driveController.getPOV("d-pad").getDirection().equals(Direction.NONE)).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 1.0)),
                new InstantCommand(() -> {driverDPadValue = driveController.getPOV("d-pad").getValue();}),
                new WaitCommand(0.25),
                new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 0.0))
            )  
        );
    }

    public void configureOperatorBindings() {
        //we dont need em;
    }

    public void configureAutoChoosers() {
        CONTAINER_CHOOSER.setDefaultOption("Red", RED_TRAJECTORIES);
        CONTAINER_CHOOSER.addOption("Blue", BLUE_TRAJECTORIES);

        AUTO_CHOOSER.setDefaultOption("Test Path", () -> new TestAutoCommand(drivetrain));
        AUTO_CHOOSER.addOption("Tune Path", () -> new TuneAutoCommand(drivetrain));
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
