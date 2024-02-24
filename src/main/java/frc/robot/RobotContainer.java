// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// Controllers
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.StressTestAuto;
import frc.robot.commands.TestAutoCommand;
import frc.robot.commands.TuneAutoCommand;
import frc.robot.commands.autos.ThreePieceClose;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.team4272.controllers.XboxController;
import frc.team4272.controllers.utilities.*;
import frc.team4272.controllers.utilities.JoystickAxes.DeadzoneMode;
import frc.team4272.controllers.utilities.JoystickPOV.Direction;

// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

// States
import frc.robot.subsystems.intake.states.*;
import frc.robot.subsystems.armelevator.states.*;
import frc.robot.subsystems.shooter.states.*;
import frc.robot.subsystems.drivetrain.states.*;

// Commands
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;

// Constants
import frc.robot.constants.Norms;
import static frc.robot.constants.AutoConstants.Paths.*;
import static frc.robot.constants.TelemetryConstants.Limelights.*;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;
import static frc.robot.constants.UniversalConstants.*;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;

import java.util.*;

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
    // Climber climber = new Climber();

    int driverDPadValue = -1;

    // The robots IO devices are defined here
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);
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
        JoystickAxes driveLeftAxes = driverController.getAxes("left");
        driveLeftAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kMagnitude).setPowerScale(3);

        JoystickAxes driveRightAxes = driverController.getAxes("right");
        driveRightAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kXAxis).setPowerScale(2.5);

        JoystickTrigger driveTriggerRight = driverController.getTrigger("right");
        driveTriggerRight.setDeadzone(0.1).setPowerScaling(2);

        JoystickTrigger driveTriggerLeft = driverController.getTrigger("left");
        driveTriggerRight.setDeadzone(0.1).setPowerScaling(2);

        //Drivetrain --------------------------------------------------------
        
        drivetrain.setDefaultCommand(
            new DriveState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, driveRightAxes::getDeadzonedX)
        );

        new Trigger(() -> !driverController.getPOV("d-pad").getDirection().equals(Direction.NONE)).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> driverController.setRumble(RumbleType.kBothRumble, 1.0)),
                new InstantCommand(() -> {driverDPadValue = driverController.getPOV("d-pad").getValue();}),
                new WaitCommand(0.12),
                new InstantCommand(() -> driverController.setRumble(RumbleType.kBothRumble, 0.0))
            )  
        );

        new Trigger(driverController.getButton("x")::get).whileTrue(
            new SelectCommand<Integer>(Map.of(
                0,
                new ParallelCommandGroup(
                    new InstantCommand(() -> driverController.setRumble(RumbleType.kBothRumble, 1.0)),
                    new PathFindToPositionState(drivetrain, AMP_POSE)
                ),
                90, 
                new GoToPositionState(drivetrain, AMP_POSE)
                ),
                () -> driverDPadValue
            )
        ).onFalse(
            new InstantCommand(() -> driverController.setRumble(RumbleType.kBothRumble, 0.0))
        );
        
        new Trigger(driverController.getButton("b")::get).onTrue(
            new ResetHeadingState(drivetrain)
        );

        new Trigger(driverController.getButton("y")::get).onTrue(
            new ResetToLimelightState(drivetrain, FRONT_LIMELIGHT)
        );

        new Trigger(driverController.getButton("a")::get).whileTrue(
            new FacePositionState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, SPEAKER_POSITION)
        );

        new Trigger(driveTriggerLeft::isTriggered).whileTrue(
            new IntakeFeedCommand(intake, shooter, driveTriggerLeft::getValue)
        );

        //Arm ----------------------------------------------------

        armElevator.setDefaultCommand(new GoToArmElevatorState(armElevator, HOME));

        new Trigger(driverController.getButton("leftBumper")::get).whileTrue(
            new AutoAimCommand(drivetrain, armElevator, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY)  
        );

        new Trigger(driveTriggerRight::isTriggered).whileTrue(
            new ShootState(shooter, driveTriggerRight::getValue, driverController.getButton("rightBumper")::get)  
        );

        new Trigger(driverController.getButton("back")::get).whileTrue(
            new GoToArmElevatorState(armElevator, TEST).repeatedly()
        );
    }

    public void configureOperatorBindings() {
        JoystickAxes operatorLeftStick = operatorController.getAxes("left");
        operatorLeftStick.setDeadzone(0.1).setPowerScale(2.0).setDeadzoneMode(DeadzoneMode.kYAxis);

        JoystickTrigger operatorLeftTrigger = operatorController.getTrigger("left");
        operatorLeftTrigger.setDeadzone(0.1).setPowerScaling(2);

        JoystickTrigger operatorRightTrigger = operatorController.getTrigger("right");
        operatorRightTrigger.setDeadzone(0.1).setPowerScaling(2);

        // climber.setDefaultCommand(
        //     new  ClimbState(climber, operatorLeftStick::getDeadzonedY)
        // );

        new Trigger(operatorController.getButton("a")::get).whileTrue(
            new OutfeedState(shooter, () -> -0.5)
        );

        new Trigger(operatorController.getButton("y")::get).onTrue(
            new ResetToLimelightState(drivetrain, FRONT_LIMELIGHT)
        );

        // new Trigger(operatorController.getButton("b")::get).whileTrue(
        //     new  ZeroElevatorState(armElevator).repeatedly()
        // );

        new Trigger(operatorController.getButton("leftBumper")::get).whileTrue(
            new ShootState(shooter, () -> 1.0, operatorController.getButton("rightBumper")::get)
        );


        new Trigger(operatorController.getPOV("d-pad")::isTriggered).whileTrue(
            new SelectCommand<Direction>(
                new EnumMap<Direction, Command>(
                    Map.of(
                        Direction.UP,
                        new GoToArmElevatorState(armElevator, AMP).repeatedly(),
                        Direction.UP_RIGHT,
                        new PrintCommand("Up Right on d-pad not assigned"),
                        Direction.RIGHT,
                        new GoToArmElevatorState(armElevator, TRAP).repeatedly(),
                        Direction.DOWN_RIGHT,
                        new PrintCommand("Down Right on d-pad not assigned"),
                        Direction.DOWN,
                        new PrintCommand("Down on d-pad not assigned"),
                        Direction.DOWN_LEFT,
                        new PrintCommand("Down Left on d-pad not assigned"),
                        Direction.LEFT,
                        new PrintCommand("Left on d-pad not assigned"),
                        Direction.UP_LEFT,
                        new PrintCommand("Up Left on d-pad not assigned")

                    )
                ),
                operatorController.getPOV("d-pad")::getDirection
            )
        );

        new Trigger(operatorLeftTrigger::isTriggered).whileTrue(
            new IntakeFeedCommand(intake, shooter, () -> 0.9)
        );

        new Trigger(operatorRightTrigger::isTriggered).whileTrue(
            new ParallelCommandGroup(
                new OuttakeState(intake, () -> 0.9),
                new OutfeedState(shooter, () -> 0.9)
            )
        );
    }

    public void configureAutoChoosers() {
        CONTAINER_CHOOSER.setDefaultOption("Red", RED_TRAJECTORIES);
        CONTAINER_CHOOSER.addOption("Blue", BLUE_TRAJECTORIES);

        AUTO_CHOOSER.setDefaultOption("Test Path", () -> new TestAutoCommand(drivetrain));
        AUTO_CHOOSER.addOption("Tune Path", () -> new TuneAutoCommand(drivetrain));
        AUTO_CHOOSER.addOption("Stress Path", () -> new StressTestAuto(drivetrain));
        AUTO_CHOOSER.addOption("Three Piece Close", () -> new ThreePieceClose(drivetrain));
        
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
