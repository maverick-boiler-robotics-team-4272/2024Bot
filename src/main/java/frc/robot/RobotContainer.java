// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

// CANdle
import frc.robot.utils.periodics.Candle;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;

// Constants
import frc.robot.constants.Norms;
import frc.robot.constants.AutoConstants.Paths;
import frc.robot.utils.periodics.CANPeriodic;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.AutoConstants.Paths.*;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.TelemetryConstants.Limelights.BACK_LIMELIGHT;
import static frc.robot.constants.TelemetryConstants.ShuffleboardTables.*;
import static frc.robot.constants.UniversalConstants.*;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;
import static frc.robot.utils.misc.BEAN.*;

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
    Candle candle = new Candle(CANDLE_ID);

    // int driverDPadValue = -1;

    // The robots IO devices are defined here
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Norms.initialize(drivetrain, armElevator);
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
                new RevAndShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans, driverController.getButton("rightBumper")::get)
                .beforeStarting(new LidarStoppedFeedState(shooter, 0.4)),
                new StartEndCommand(() -> {
                    driverController.setRumble(RumbleType.kBothRumble, 1.0);
                }, () -> {
                    driverController.setRumble(RumbleType.kBothRumble, 0.0);
                }).beforeStarting(new WaitCommand(1.0))
            )
        );

        new Trigger(driverController.getButton("a")::get).whileTrue(
            new RotLockState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, getGlobalPositions().AMP_POSE::getRotation)
        );

        new Trigger(driverController.getButton("x")::get).whileTrue(
            // new RotLockState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, () -> getGlobalPositions().TO_SOURCE)
            // new NoteLockState(drivetrain, intake, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY)
            //// new AutoNotePickup(drivetrain, intake, shooter, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY)
            // .alongWith(new IntakeFeedCommand(intake, shooter, 0.8))
            drivetrain.pathFind(getGlobalPositions().AMP_POSE)
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

        // new Trigger(driverController.getButton("y")::get).onTrue(
        //     new ResetToLimelightState(drivetrain, FRONT_LIMELIGHT)
        // );

        new Trigger(driverController.getButton("back")::get).whileTrue(
            new InstantCommand(drivetrain::resetModules, drivetrain)
        );

        new Trigger(driverController.getButton("start")::get).whileTrue(
            new InstantCommand(drivetrain::enableVisionFusion)
        );

        //Arm ----------------------------------------------------

        armElevator.setDefaultCommand(new GoToArmElevatorState(armElevator, HOME));
    }

    private void configureOperatorBindings() {
        JoystickAxes operatorLeftStick = operatorController.getAxes("left");
        operatorLeftStick.setDeadzone(0.1).setPowerScale(2.0).setDeadzoneMode(DeadzoneMode.kYAxis);

        JoystickTrigger operatorLeftTrigger = operatorController.getTrigger("left");
        operatorLeftTrigger.setDeadzone(0.1).setPowerScaling(2);

        JoystickTrigger operatorRightTrigger = operatorController.getTrigger("right");
        operatorRightTrigger.setDeadzone(0.1).setPowerScaling(2);

        // JoystickAxes driveLeftAxes = driverController.getAxes("left");

        JoystickPOV operatorDPad = operatorController.getPOV("d-pad");

        // new Trigger(operatorController.getButton("start")::get).onTrue(
        //     new RotLockState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, () -> getGlobalPositions().TO_SOURCE)
        // );

        new Trigger(operatorController.getButton("a")::get).and(() -> !operatorController.getButton("x").get()).whileTrue(
            new ImbalancedShootState(shooter, PINTO_BEAN.beans, STRING_BEAN.beans, 0.2)
        );
            
        new Trigger(operatorController.getButton("x")::get).whileTrue(
            new ParallelRaceGroup(
                new RevAndShootState(shooter, CANNELLINI_BEAN.beans, BAKED_BEAN.beans, false, driverController.getButton("rightBumper")::get),
                new GoToArmElevatorState(armElevator, WHITE_LINE).repeatedly()
            )
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

        JoystickAxes driveLeftAxes = driverController.getAxes("left");

        new Trigger(operatorController.getButton("rightBumper")::get).whileTrue(
            new GoToArmElevatorState(armElevator, SOURCE).repeatedly()
        );

        new Trigger(operatorLeftTrigger::isTriggered).whileTrue(
            new SequentialCommandGroup(
                new IntakeFeedCommand(intake, shooter, GARBANZO_BEAN.beans).until(shooter::beginLidarTripped),
                new ScheduleCommand(
                    new IntakeFeedCommand(intake, shooter, GARBANZO_BEAN.beans)
                )
            ).alongWith(
                new ConditionalCommand(
                    new AutoNotePickup(drivetrain, intake, shooter, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY), 
                    new DriveState(drivetrain, driveLeftAxes::getDeadzonedX, driveLeftAxes::getDeadzonedY, () -> { return driverController.getAxes("right").getDeadzonedX(); } ), 
                    BACK_LIMELIGHT::getTV
                )
            )
        );
        
        new Trigger(operatorLeftTrigger::isTriggered).and(operatorController.getButton("back")::get).whileTrue(
            new ParallelCommandGroup(
                new IntakeState(intake, GARBANZO_BEAN.beans),
                new FeedState(shooter, GARBANZO_BEAN.beans)
            )
        );

        
        new Trigger(operatorLeftTrigger::isTriggered).and(operatorController.getButton("rightBumper")::get).whileTrue(
            new ShootState(shooter, PINTO_BEAN.stalk().beans, PINTO_BEAN.stalk().beans) {
                public boolean isFinished() {
                    return shooter.beginLidarTripped();
                }
            }.andThen(
                new LidarStoppedFeedState(shooter, PINTO_BEAN.beans)
            )
        );

        new Trigger(operatorRightTrigger::isTriggered).whileTrue(
            new ParallelCommandGroup(
                new OuttakeState(intake, GARBANZO_BEAN.beans),
                new OutfeedState(shooter, GARBANZO_BEAN.beans)
            )
        );
        
        new Trigger(operatorController.getButton("leftBumper")::get).whileTrue(
            new RevAndShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans, driverController.getButton("rightBumper")::get)
        );

        AUTO_TABLE.putBoolean("Is Latched", false).withSize(5, 5);

        new Trigger(operatorController.getButton("leftStick")::get).toggleOnTrue(
            new ParallelCommandGroup(
                new LatchState(armElevator),
                new StartEndCommand(() -> {
                    candle.setLEDs(0, 0, 255);
                    AUTO_TABLE.putBoolean("Is Latched", true);
                }, () -> {
                    candle.setLEDs(0, 0, 0);
                    AUTO_TABLE.putBoolean("Is Latched", false);
                })
            )
        );

        new Trigger(operatorController.getButton("y")::get).whileTrue(
            new ClimbState(armElevator, operatorLeftStick::getDeadzonedY)
        );

        new Trigger(operatorController.getButton("b")::get).whileTrue(
            new ZeroElevatorState(armElevator)
        );
    }

    private void configureAutoChoosers() {
        CONTAINER_CHOOSER.setDefaultOption("Red", "Red");
        CONTAINER_CHOOSER.addOption("Blue", "Blue");

        AUTO_CHOOSER.addOption("Test", new PathPlannerAuto("Test"));
        AUTO_CHOOSER.setDefaultOption("8123", new PathPlannerAuto("8123"));
        AUTO_CHOOSER.addOption("P45", new PathPlannerAuto("P45"));
        
        AUTO_TABLE.putData("Auto Chooser", AUTO_CHOOSER);
        AUTO_TABLE.putData("Side Chooser", CONTAINER_CHOOSER).withWidget(BuiltInWidgets.kSplitButtonChooser);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("UnFace", new InstantCommand(drivetrain::rotateToPath));
        NamedCommands.registerCommand("AimShoot", 
            new InstantCommand(drivetrain::overrideRotation).andThen(
                new AutoShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans).raceWith(
                    Commands.defer(() -> new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), getGlobalPositions().SPEAKER_TARGET_POSITION), Set.of(armElevator))
                ).andThen(new InstantCommand(drivetrain::rotateToPath))
            )
        );

        NamedCommands.registerCommand("LongAimShoot", 
            new InstantCommand(drivetrain::overrideRotation).andThen(
                new AutoShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans, 1.00).raceWith(
                    Commands.defer(() -> new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), getGlobalPositions().SPEAKER_TARGET_POSITION), Set.of(armElevator))
                ).andThen(new InstantCommand(drivetrain::rotateToPath))
            )
        );

        NamedCommands.registerCommand("Intake", new IntakeFeedCommand(intake, shooter, BAKED_BEAN.beans).withTimeout(7.5));
        NamedCommands.registerCommand("Disable", new InstantCommand(drivetrain::disableVisionFusion));
        NamedCommands.registerCommand("Enable", new InstantCommand(drivetrain::enableVisionFusion));
        // NamedCommands.registerCommand("AutoAim", Commands.defer(() -> new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), getGlobalPositions().SPEAKER_TARGET_POSITION), Set.of(armElevator)));
        NamedCommands.registerCommand("AutoShoot", new ParallelRaceGroup(
            Commands.defer(() -> new AutoAimCommand(drivetrain, armElevator, () -> JELLY_BEAN.beans, () -> JELLY_BEAN.beans), Set.of(drivetrain, armElevator)),
            new AutoShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans)
        ));

        NamedCommands.registerCommand("LongAutoShoot", new ParallelRaceGroup(
            Commands.defer(() -> new AutoAimCommand(drivetrain, armElevator, () -> JELLY_BEAN.beans, () -> JELLY_BEAN.beans), Set.of(drivetrain, armElevator)),
            new AutoShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans, 0.75)
        ));

        NamedCommands.registerCommand("Face", new InstantCommand(drivetrain::overrideRotation));

        NamedCommands.registerCommand("Line", new GoToArmElevatorState(armElevator, START_LINE));
        NamedCommands.registerCommand("Drop", new SequentialCommandGroup(
            // new GoToArmElevatorState(armElevator, ArmElevatorSetpoint.createArbitrarySetpoint(Units.Meters.convertFrom(5.0, Units.Inches), new Rotation2d(0.0))),
            new GoToArmElevatorState(armElevator, HOME),
            new RevAndShootState(shooter, 50, BAKED_BEAN.beans, () -> true).withTimeout(1.0)
        ));

        NamedCommands.registerCommand("AimAutoShoot", new SequentialCommandGroup(
            new ParallelCommandGroup(
                Commands.defer(() -> new AngleStoppedAutoAimCommand(drivetrain, armElevator, () -> JELLY_BEAN.beans, () -> JELLY_BEAN.beans), Set.of(drivetrain, armElevator)),
                new RevState(shooter, BAKED_BEAN.beans).withTimeout(0.75)
            ),
            new RevAndShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans, () -> true).withTimeout(0.5)
        ));

        NamedCommands.registerCommand("Subshot", new GoToArmElevatorState(armElevator, SUB_SHOT).alongWith(new AutoShootState(shooter, 0.9, 0.9)).andThen(new GoToArmElevatorState(armElevator, HOME)));
        
        NamedCommands.registerCommand("Home", new GoToArmElevatorState(armElevator, HOME));
        NamedCommands.registerCommand("Reset", new InstantCommand(drivetrain::resetModules, drivetrain));

        NamedCommands.registerCommand("Index", new LidarStoppedFeedState(shooter, BAKED_BEAN.beans, LIMA_BEAN.beans));
        NamedCommands.registerCommand("DriveBy", new ParallelCommandGroup(
            Commands.defer(() -> new AngleStoppedAutoAimCommand(drivetrain, armElevator, () -> JELLY_BEAN.beans, () -> JELLY_BEAN.beans), Set.of(armElevator)),
            new RevAndShootState(shooter, BAKED_BEAN.beans, BAKED_BEAN.beans, () -> true),
            new IntakeState(intake, 0.95)
        ));
    }

    private void configureSignalingBindings() {
        Animation rainbowAnimation = new RainbowAnimation(1.0, 0.5, 512);
        
        new Trigger(shooter::lidarTripped).onTrue(
            new InstantCommand(() -> {
                candle.animate(rainbowAnimation);
            }).ignoringDisable(true)
        ).onFalse(
            new InstantCommand(() -> {
                candle.animate(null);
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

        new Trigger(() -> armElevator.getElevatorDesiredHeight() == HOME.getElevatorHeight())
            .and(() -> armElevator.getShooterDesiredRotation() == HOME.getArmAngle().getRadians())
            .and(() -> armElevator.getElevatorMotorCurrent() > 15.0)
            .and(() -> Math.abs(armElevator.getElevatorHeight() - armElevator.getElevatorDesiredHeight()) < Meters.convertFrom(2.0, Inches))
            .onTrue(
                new InstantCommand(() -> {
                    operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                })
            ).onFalse(
                new InstantCommand(() -> {
                    operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                })
            );
    }

    private void addResetButtons() {
        OVERRIDE_TABLE.putData("Reset Intake Motor", new InstantCommand(intake::resetIntakeMotor, intake));
        OVERRIDE_TABLE.putData("Reset Shooter Motors", new InstantCommand(shooter::resetShooterMotors, shooter));
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

        OVERRIDE_TABLE.putBoolean("Auto Note Intake", false).withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1);
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

        return AUTO_CHOOSER.getSelected();
    }
}
