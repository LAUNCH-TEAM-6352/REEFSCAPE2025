// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants.ClimberKeys;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.test.TestDriveTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climb;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems:
    private final Optional<DriveTrain> driveTrain;
    private final Optional<Climber> climber;

    // OI devices:

    private final XboxController driverGamepad;
    private final XboxController codriverGamepad;

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
        }
        else
        {
            // In competition, don't take chances and always create all OI devices:
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
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        climber.ifPresent(this::configureBindings);

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.

    }

    private void configureBindings(Climber climber)
    {
        if (codriverGamepad != null)
        {
            return;
        }
        new JoystickButton(codriverGamepad, Button.kB.value).and(
            new JoystickButton(codriverGamepad, Button.kA.value))
                .whileTrue(new Climb(climber, ClimberKeys.winchMotorSpeedKey));
    }

    private void configureSmartDashboard()
    {
        driveTrain.ifPresent(this::configureSmartDashboard);
        climber.ifPresent(this::configureSmartDashboard);

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
        // An example command will be run in autonomous
        return null;
    }

    /**
     * Builds a command to run when Test mode is enabled in the Driver Station.
     */
    public Command getTestCommand()
    {
        var group = new SequentialCommandGroup();

        if (driveTrain.isPresent())
        {
            group.addCommands(new TestDriveTrain(driveTrain.get()));
        }

        return group;
    }
}
