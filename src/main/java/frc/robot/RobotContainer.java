// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.DashboardConstants.ClimberKeys;
import frc.robot.Constants.DashboardConstants.CoralManipulatorKeys;
import frc.robot.Constants.DashboardConstants.ElevatorKeys;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TestConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.MoveElevatorWithGamepad;
import frc.robot.commands.test.TestCoralManipulator;
import frc.robot.commands.test.TestDriveTrain;
import frc.robot.commands.test.TestElevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.CoralReceiver;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems:
    // May need tyo make driveTrain private again
    private final Optional<DriveTrain> driveTrain;
    private final Optional<Climber> climber;
    private final Optional<CoralManipulator> coralManipulator;
    private final Optional<Elevator> elevator;
    private final Optional<CoralReceiver> coralReceiver;
    private final Optional<Limelight> limelight;

    // OI devices:

    private final XboxController driverGamepad;
    private final XboxController codriverGamepad;
    private final CommandXboxController commandCodriverGamepad;

    SendableChooser<Boolean> driveOrientationChooser = new SendableChooser<>();
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Get the game data message fom the driver station.
        // This message is primarily used during development to
        // construct only certain subsystems.
        // If the message is blank (or all whitespace),
        // all subsystems are constructed.
        // Otherwise, OI devices and subsystems are constructed
        // depending upon the substrings found in the message:
        // -dt- Drive train
        // -oi- OI devices
        // -cl- Climber
        // -cm- Coral manipulator
        // -e- Elevator
        // -cr- Coral receiver
        // -l- Limelight

        var gameData = DriverStation.getGameSpecificMessage().toLowerCase();
        SmartDashboard.putString("Game Data", gameData);

        // Create OI devices:
        if (gameData.contains("-oi-"))
        {
            // Explicitly look for OI devices:
            driverGamepad = DriverStation.isJoystickConnected(OperatorConstants.driverGamepadPort)
                ? new XboxController(OperatorConstants.driverGamepadPort)
                : null;
            codriverGamepad = DriverStation.isJoystickConnected(OperatorConstants.codriverGamepadPort)
                ? new XboxController(OperatorConstants.codriverGamepadPort)
                : null;
            commandCodriverGamepad = DriverStation.isJoystickConnected(OperatorConstants.codriverGamepadPort)
                ? new CommandXboxController(OperatorConstants.codriverGamepadPort)
                : null;
        }
        else
        {
            // In competition, don't take chances and always create all OI devices:
            commandCodriverGamepad = new CommandXboxController(OperatorConstants.codriverGamepadPort);
            codriverGamepad = new XboxController(OperatorConstants.codriverGamepadPort);
            driverGamepad = new XboxController(OperatorConstants.driverGamepadPort);
        }

        // Create subsystems:
        driveTrain = gameData.isBlank() || gameData.contains("-dt-")
            ? Optional.of(new DriveTrain())
            : Optional.empty();

        climber = gameData.isBlank() || gameData.contains("-cl-")
            ? Optional.of(new Climber())
            : Optional.empty();

        coralManipulator = gameData.isBlank() || gameData.contains("-cm-")
            ? Optional.of(new CoralManipulator())
            : Optional.empty();

        elevator = gameData.isBlank() || gameData.contains("-e-")
            ? Optional.of(new Elevator())
            : Optional.empty();

        coralReceiver = gameData.isBlank() || gameData.contains("-cr-")
            ? Optional.of(new CoralReceiver())
            : Optional.empty();

        limelight = gameData.isBlank() || gameData.contains("-l-")
            ? Optional.of(new Limelight())
            : Optional.empty();

        // Configure commands for Path Planner:
        configurePathPlannerNamedCommands();

        // Configure default commands
        configureDefaultCommands();

        // Configure the trigger bindings
        configureBindings();

        // Configure smart dashboard
        configureSmartDashboard();
    }

    /**
     * Configure named commands for Path Planner.
     */
    private void configurePathPlannerNamedCommands()
    {
        // EX: NamedCommands.registerCommand("Shoot Wait", new Wait(AutoKeys.shootWaitTime));
    }

    /**
     * Configures the default commands.
     */
    private void configureDefaultCommands()
    {
        driveTrain.ifPresent((dt) ->
        {
            if (driverGamepad != null)
            {
                dt.setDefaultCommand(new DriveWithGamepad(dt, driverGamepad, driveOrientationChooser));
            }
        });
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        climber.ifPresent(this::configureBindings);
        coralManipulator.ifPresent(this::configureBindings);
        elevator.ifPresent(this::configureBindings);
        coralReceiver.ifPresent(this::configureBindings);

        if (driveTrain.isPresent() && limelight.isPresent())
        {
            configureBindings(driveTrain.get(), limelight.get());
        }
    }

    private void configureBindings(CoralManipulator coralManipulator)
    {
        if (commandCodriverGamepad == null)
        {
            return;
        }

        commandCodriverGamepad.leftBumper()
            .onTrue(new InstantCommand(() -> coralManipulator.intakeCoral()));

        commandCodriverGamepad.rightBumper()
            .onTrue(new InstantCommand(() -> coralManipulator.ejectCoral()));
    }

    private void configureBindings(CoralReceiver coralReceiver)
    {
        if (commandCodriverGamepad == null)
        {
            return;
        }

        commandCodriverGamepad.rightTrigger()
            .onTrue(new InstantCommand(() -> coralReceiver.moveUp()));
    }

    private void configureBindings(Climber climber)
    {
        if (commandCodriverGamepad == null || codriverGamepad == null)
        {
            return;
        }

        // TODO: Need to move coral trey out of the way before climbing,
        // but don't want to make moving the coral tray part of the whileTrue().
        commandCodriverGamepad.start().and(commandCodriverGamepad.back())
            .whileTrue(new Climb(climber, ClimberKeys.winchMotorSpeedKey, codriverGamepad));
    }

    private void configureBindings(DriveTrain driveTrain, Limelight limelight)
    {
        if (commandCodriverGamepad == null)
        {
            return;
        }

        commandCodriverGamepad.b().onTrue(new DriveToTarget(limelight, driveTrain));
    }

    private void configureBindings(Elevator elevator)
    {
        if (commandCodriverGamepad == null || codriverGamepad == null)
        {
            return;
        }
        commandCodriverGamepad.rightStick()
            .onTrue(new MoveElevatorWithGamepad(elevator, codriverGamepad));

        commandCodriverGamepad.a()
            .onTrue(new MoveElevatorToPosition(elevator, codriverGamepad, ElevatorKeys.toleranceKey));
    }

    private void configureSmartDashboard()
    {
        driveTrain.ifPresent(this::configureSmartDashboard);
        climber.ifPresent(this::configureSmartDashboard);
        coralManipulator.ifPresent(this::configureSmartDashboard);
        elevator.ifPresent(this::configureSmartDashboard);

        // Configure chooser widgets:
        configureDriveOrientationChooser(driveOrientationChooser);
    }

    private void configureSmartDashboard(DriveTrain driveTrain)
    {
    }

    private void configureSmartDashboard(Climber climber)
    {
        SmartDashboard.putNumber(ClimberKeys.winchMotorSpeedKey, ClimberConstants.winchMotorSpeed);

    }

    private void configureSmartDashboard(CoralManipulator coralManipulator)
    {
        SmartDashboard.putNumber(CoralManipulatorKeys.rollerMotorIntakeSpeedKey,
            CoralManipulatorConstants.rollerMotorIntakeSpeed);
        SmartDashboard.putNumber(CoralManipulatorKeys.rollerMotorBackupSpeedKey,
            CoralManipulatorConstants.rollerMotorBackupSpeed);
        SmartDashboard.putNumber(CoralManipulatorKeys.rollerMotorEjectSpeedKey,
            CoralManipulatorConstants.rollerMotorEjectSpeed);
        SmartDashboard.putNumber(CoralManipulatorKeys.opticalSensorVoltageThresholdKey,
            CoralManipulatorConstants.opticalSensorVoltageThreshold);
        SmartDashboard.putNumber(CoralManipulatorKeys.extraTimeSecsKey, CoralManipulatorConstants.extraTimeSecs);
    }

    private void configureSmartDashboard(Elevator elevator)
    {
        SmartDashboard.putNumber(ElevatorKeys.toleranceKey, ElevatorConstants.PIDConstants.tolerance);

    }

    private void configureDriveOrientationChooser(SendableChooser<Boolean> driveOrientationChooser)
    {
        driveOrientationChooser.setDefaultOption("Field Relative", Boolean.TRUE);
        driveOrientationChooser.addOption("Robot Relative", Boolean.FALSE);
        SmartDashboard.putData("Drive Orientation", driveOrientationChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }

    /**
     * Builds a command to run when Test mode is enabled in the Driver Station.
     */
    public Command getTestCommand()
    {
        var group = new SequentialCommandGroup();

        // Wait for startup messages to be logged to driver station console:
        group.addCommands(new WaitCommand(5));

        if (driveTrain.isPresent())
        {
            group.addCommands(new TestDriveTrain(driveTrain.get()));
        }

        if (elevator.isPresent())
        {
            group.addCommands(new TestElevator(elevator.get()));
        }

        if (coralManipulator.isPresent())
        {
            group.addCommands(
                new TestCoralManipulator(coralManipulator.get(),
                    SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorIntakeSpeedKey,
                        CoralManipulatorConstants.rollerMotorIntakeSpeed))
                            .withTimeout((TestConstants.coralManipulatorTimeoutSecs)),

                new TestCoralManipulator(coralManipulator.get(),
                    SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorBackupSpeedKey,
                        CoralManipulatorConstants.rollerMotorBackupSpeed))
                            .withTimeout((TestConstants.coralManipulatorTimeoutSecs)),

                new TestCoralManipulator(coralManipulator.get(),
                    SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorEjectSpeedKey,
                        CoralManipulatorConstants.rollerMotorEjectSpeed))
                            .withTimeout((TestConstants.coralManipulatorTimeoutSecs)));
        }

        return group;
    }
}
