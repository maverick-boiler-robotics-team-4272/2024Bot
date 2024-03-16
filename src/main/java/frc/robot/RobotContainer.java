// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.commands.autos.*;

// Constants
import frc.robot.constants.Norms;
import frc.robot.constants.AutoConstants.Paths;
import frc.robot.utils.periodics.CANPeriodic;
import frc.robot.utils.periodics.Candle;

import static frc.robot.constants.AutoConstants.Paths.*;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.TelemetryConstants.Limelights.*;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;
import static frc.robot.constants.UniversalConstants.*;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;

import java.util.*;

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
    IntakeSubsystem intake = new IntakeSubsystem();
    ArmElevatorSubsystem armElevator = new ArmElevatorSubsystem();
    Shooter shooter = new Shooter();
    Candle candle = new Candle(CANDLE_ID);

    // int driverDPadValue = -1;

    // The robots IO devices are defined here
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Norms.initialize(drivetrain);
        CANPeriodic.setUpLogging();

        // Configure the trigger bindings
        configureBindings();

        registerNamedCommands();
        
        Paths.initializeTrajectories();
        configureAutoChoosers();

        configureSignalingBindings();

        addResetButtons();
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

    public void configureRuntimeDriverBindings() {
        JoystickAxes driveLeftAxes = driverController.getAxes("left");

        new Trigger(driverController.getButton("rightStick")::get).whileTrue(
            new FacePositionState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, getGlobalPositions().SPEAKER_POSITION)
        );

        new Trigger(driverController.getButton("leftBumper")::get).whileTrue(
            new ParallelCommandGroup(
                new AutoAimCommand(drivetrain, armElevator, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY), 
                new SequentialCommandGroup(
                    new ShootState(shooter, 1.0, 0.0) {
                        public boolean isFinished() {
                            return driverController.getButton("rightBumper").get();
                        }
                    },
                    new ShootState(shooter, 1.0, 1.0)
                )
            )
        );

        new Trigger(driverController.getButton("a")::get).whileTrue(
            new RotLockState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, getGlobalPositions().AMP_POSE::getRotation)
        );

        new Trigger(driverController.getButton("x")::get).whileTrue(
            new RotLockState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, () -> getGlobalPositions().TO_SOURCE)
        );
    }

    private void configureDriverBindings() {
        JoystickAxes driveLeftAxes = driverController.getAxes("left");
        driveLeftAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kMagnitude).setPowerScale(3);

        JoystickAxes driveRightAxes = driverController.getAxes("right");
        driveRightAxes.setDeadzone(0.1).setDeadzoneMode(DeadzoneMode.kXAxis).setPowerScale(2.5);

        JoystickTrigger driveTriggerRight = driverController.getTrigger("right");
        driveTriggerRight.setDeadzone(0.1).setPowerScaling(2);

        JoystickTrigger driveTriggerLeft = driverController.getTrigger("left");
        driveTriggerLeft.setDeadzone(0.1).setPowerScaling(2);

        //Drivetrain --------------------------------------------------------
        
        drivetrain.setDefaultCommand(
            new DriveState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, driveRightAxes::getDeadzonedX)
        );
        
        new Trigger(driverController.getButton("b")::get).onTrue(
            new ResetHeadingState(drivetrain)
        );

        new Trigger(driverController.getButton("y")::get).onTrue(
            new ResetToLimelightState(drivetrain, FRONT_LIMELIGHT)
        );

        new Trigger(driverController.getButton("back")::get).whileTrue(
            new InstantCommand(drivetrain::resetModules, drivetrain)
        );

        new Trigger(driverController.getButton("start")::get).whileTrue(
            new RevAndShootState(shooter, 0.35, 0.5, false, driveTriggerRight::isTriggered)
        );

        //Arm ----------------------------------------------------

        //TODO: Fix this
        new Trigger(driveTriggerLeft::isTriggered).whileTrue(
            new SequentialCommandGroup(
                new LidarStoppedFeedState(shooter, 0.4),
                new ParallelRaceGroup(
                    new RevAndShootState(shooter, 0.45, 1.0, false, driveTriggerRight::isTriggered),
                    new GoToArmElevatorState(armElevator, WHITE_LINE).repeatedly()
                )
            )
        );

        armElevator.setDefaultCommand(new GoToArmElevatorState(armElevator, HOME));

    }

    private void configureOperatorBindings() {
        JoystickAxes operatorRightStick = operatorController.getAxes("right");
        operatorRightStick.setDeadzone(0.1).setPowerScale(2.0).setDeadzoneMode(DeadzoneMode.kYAxis);

        JoystickTrigger operatorLeftTrigger = operatorController.getTrigger("left");
        operatorLeftTrigger.setDeadzone(0.1).setPowerScaling(2);

        JoystickTrigger operatorRightTrigger = operatorController.getTrigger("right");
        operatorRightTrigger.setDeadzone(0.1).setPowerScaling(2);

        JoystickPOV operatorDPad = operatorController.getPOV("d-pad");

        new Trigger(operatorController.getButton("a")::get).whileTrue(
            new ImbalancedShootState(shooter, 0.50, 0.05, 0.2)
        );

        new Trigger(operatorController.getButton("y")::get).onTrue(
            new ResetToLimelightState(drivetrain, FRONT_LIMELIGHT)
        );
            
        new Trigger(operatorController.getButton("x")::get).whileTrue(
            new GoToArmElevatorState(armElevator, TRAP).repeatedly()
        );

        new Trigger(operatorDPad::isTriggered).whileTrue(
            new SelectCommand<Direction>(
                new EnumMap<Direction, Command>(
                    Map.of(
                        Direction.UP,
                        new GoToArmElevatorState(armElevator, AMP).repeatedly(),
                        Direction.UP_RIGHT,
                        new PrintCommand("Up Right on d-pad not assigned"),
                        Direction.RIGHT,
                        new GoToArmElevatorState(armElevator, SUB_SHOT).repeatedly(),
                        Direction.DOWN_RIGHT,
                        new PrintCommand("Down Right on d-pad not assigned"),
                        Direction.DOWN,
                        new GoToArmElevatorState(armElevator, PODIUM).repeatedly(),
                        Direction.DOWN_LEFT,
                        new PrintCommand("Down Left on d-pad not assigned"),
                        Direction.LEFT,
                        new GoToArmElevatorState(armElevator, WHITE_LINE).repeatedly(),
                        Direction.UP_LEFT,
                        new PrintCommand("Up Left on d-pad not assigned")

                    )
                ),
                operatorDPad::getDirection
            )
        );

        new Trigger(operatorController.getButton("rightBumper")::get).whileTrue(
            new GoToArmElevatorState(armElevator, SOURCE).repeatedly()
        );

        new Trigger(operatorLeftTrigger::isTriggered).whileTrue(
            new SequentialCommandGroup(
                new IntakeFeedCommand(intake, shooter, 0.9).until(shooter::beginLidarTripped),
                new ScheduleCommand(
                    new IntakeFeedCommand(intake, shooter, 0.9)
                )
            )
        );
        
        new Trigger(operatorLeftTrigger::isTriggered).and(operatorController.getButton("back")::get).whileTrue(
            new ParallelCommandGroup(
                new IntakeState(intake, 0.9),
                new FeedState(shooter, 0.9)
            )
        );

        
        new Trigger(operatorLeftTrigger::isTriggered).and(operatorController.getButton("rightBumper")::get).whileTrue(
            new ShootState(shooter, -0.5, -0.5) {
                public boolean isFinished() {
                    return shooter.beginLidarTripped();
                }
            }.andThen(
                new LidarStoppedFeedState(shooter, 0.5)
            )
        );

        new Trigger(operatorRightTrigger::isTriggered).whileTrue(
            new ParallelCommandGroup(
                new OuttakeState(intake, 0.9),
                new OutfeedState(shooter, 0.9)
            )
        );
        
        new Trigger(operatorController.getButton("leftBumper")::get).whileTrue(
            new SequentialCommandGroup(
                new ShootState(shooter, 1.0, 0.0) {
                    public boolean isFinished() {
                        return driverController.getButton("rightBumper").get();
                    }
                },
                new ShootState(shooter, 1.0, 1.0)
            )
        );

    }

    private void configureAutoChoosers() {
        CONTAINER_CHOOSER.setDefaultOption("Red", "Red");
        CONTAINER_CHOOSER.addOption("Blue", "Blue");

        AUTO_CHOOSER.addOption("P28", () -> new TwoEight(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P16", () -> new OneSix(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P14", () -> new OneFourRush(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P45", () -> new FourFive(drivetrain));
        AUTO_CHOOSER.addOption("P123", () -> new OneTwoThree(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P123Plus", () -> new OneTwoThreePlus(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P1238", () -> new OneTwoThreeEight(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P1238Plus", () -> new OneTwoThreeEightPlus(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P1238PlusTest", () -> new OneTwoThreePlusTwo(drivetrain, armElevator, shooter));
        AUTO_CHOOSER.addOption("P two Any", () -> new TwoPiece(drivetrain, armElevator, shooter, intake));
        AUTO_CHOOSER.addOption("P Shoot", () -> new FireAndSit(drivetrain, armElevator, shooter));
        
        AUTO_TABLE.putData("Auto Chooser", AUTO_CHOOSER);
        AUTO_TABLE.putData("Side Chooser", CONTAINER_CHOOSER).withWidget(BuiltInWidgets.kSplitButtonChooser);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Shoot", new AutoShootState(shooter, 1, 1));
        NamedCommands.registerCommand("Intake", new IntakeFeedCommand(intake, shooter, 1.0).withTimeout(7.5));
        NamedCommands.registerCommand("Disable", new InstantCommand(drivetrain::disableVisionFusion));
        NamedCommands.registerCommand("Enable", new InstantCommand(drivetrain::enableVisionFusion));
        NamedCommands.registerCommand("AutoAim", Commands.defer(() -> new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0), Set.of(drivetrain, armElevator)));
        NamedCommands.registerCommand("AutoShoot", new ParallelRaceGroup(
            Commands.defer(() -> new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0), Set.of(drivetrain, armElevator)),
            new AutoShootState(shooter, 1.0, 1.0)//.beforeStarting(
            //     new WaitCommand(0.25)
            // )
        ));

        NamedCommands.registerCommand("Index", new LidarStoppedFeedState(shooter, 1.0, 0.1));

    }

    private void configureSignalingBindings() {
        new Trigger(shooter::lidarTripped).onTrue(
            new InstantCommand(() -> {
                candle.setLEDs(255, 192, 203);
            }).ignoringDisable(true)
        ).onFalse(
            new InstantCommand(() -> {
                candle.setLEDs(0, 0, 0);
            }).ignoringDisable(true)
        );

        new Trigger(intake::isMotorStalling).onTrue(
            new InstantCommand(() -> {
                candle.setLEDs(255, 0, 0);
            })
        ).onFalse(
            new InstantCommand(() -> {
                candle.setLEDs(0, 0, 0);
            })
        );
    }

    private void addResetButtons() {
        OVERRIDE_TABLE.putData("Reset Intake Motor", new InstantCommand(intake::resetIntakeMotor, intake));
        OVERRIDE_TABLE.putData("Reset Shooter Meotors", new InstantCommand(shooter::resetShooterMotors, shooter));
        OVERRIDE_TABLE.putData("Reset Feed Motor", new InstantCommand(shooter::resetFeedMotor, shooter));
        OVERRIDE_TABLE.putData("Reset Arm Motor", new InstantCommand(armElevator::resetArmMotor, armElevator));
        OVERRIDE_TABLE.putData("Reset Swerve Modules", new InstantCommand(drivetrain::resetModules, drivetrain));
        OVERRIDE_TABLE.putData("Reset All", new InstantCommand(() -> {
            intake.resetIntakeMotor();
            shooter.resetShooterMotors();
            shooter.resetFeedMotor();
            armElevator.resetArmMotor();
            drivetrain.resetModules();
        }, intake, shooter, armElevator, drivetrain));
        OVERRIDE_TABLE.putData("Reset All But Arm", new InstantCommand(() -> {
            intake.resetIntakeMotor();
            shooter.resetShooterMotors();
            shooter.resetFeedMotor();
            drivetrain.resetModules();
        }, intake, shooter, drivetrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if(!hasGlobalTrajectories()) {
            if(CONTAINER_CHOOSER.getSelected().equals("Red")) {
                setGlobalTrajectories(Paths.getRedTrajectories());
                setGlobalPositions(RED_POSITIONS);
            } else {
                setGlobalTrajectories(Paths.getBlueTrajectories());
                setGlobalPositions(BLUE_POSITIONS);
            }
        }

        return AUTO_CHOOSER.getSelected().get();
    }
}
