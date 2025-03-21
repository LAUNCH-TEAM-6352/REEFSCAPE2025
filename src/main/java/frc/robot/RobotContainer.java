// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.DashboardConstants.ClimberKeys;
import frc.robot.Constants.DashboardConstants.CoralManipulatorKeys;
import frc.robot.Constants.DashboardConstants.ElevatorKeys;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TestConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.MoveAlgaeManipulatorWithGamepad;
import frc.robot.commands.MoveElevatorToCoralPosition;
import frc.robot.commands.MoveElevatorWithGamepad;
import frc.robot.commands.test.TestAlgaeManipulator;
import frc.robot.commands.test.TestClimber;
import frc.robot.commands.test.TestCoralManipulator;
import frc.robot.commands.test.TestCoralReceiver;
import frc.robot.commands.test.TestDriveTrain;
import frc.robot.commands.test.TestElevator;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.CoralReceiver;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems:
    protected final Optional<DriveTrain> driveTrain;
    private final Optional<Climber> climber;
    private final Optional<CoralManipulator> coralManipulator;
    private final Optional<Elevator> elevator;
    private final Optional<CoralReceiver> coralReceiver;
    private final Optional<AlgaeManipulator> algaeManipulator;

    // OI devices:
    private Joystick driverJoystick = null;
    private XboxController driverGamepad = null;
    private final XboxController codriverGamepad;
    private final CommandXboxController commandCodriverGamepad;

    // Drive configuration parameters:
    private boolean isDrivingFieldRelative;

    SendableChooser<Boolean> driveOrientationChooser = new SendableChooser<>();
    SendableChooser<Command> autoChooser = new SendableChooser<>();
    SendableChooser<Command> driverHIDChooser = new SendableChooser<>();

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
        // -am- AlgaeManipulator

        var gameData = DriverStation.getGameSpecificMessage().toLowerCase();
        SmartDashboard.putString("Game Data", gameData);

        // Create OI devices:
        if (gameData.contains("-oi-"))
        {
            // Explicitly look for OI devices:
            driverJoystick = DriverStation.isJoystickConnected(OperatorConstants.driverJoystickPort)
                ? new Joystick(OperatorConstants.driverJoystickPort)
                : null;
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
        algaeManipulator = gameData.isBlank() || gameData.contains("-am-")
            ? Optional.of(new AlgaeManipulator())
            : Optional.empty();

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
        NamedCommands.registerCommand("Score L4", new SequentialCommandGroup(
            new InstantCommand(() -> elevator.get().setPosition(CoralLevel.Reef4.elevatorPosition(), ElevatorConstants.PIDConstants.tolerance)),
            new WaitCommand(TestConstants.instantInBetweenSecs),
            new InstantCommand(() -> coralManipulator.get().ejectCoral())));
    }

    /**
     * Configures the default commands.
     */
    private void configureDefaultCommands()
    {
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
        algaeManipulator.ifPresent(this::configureBindings);
    }

    private void configureBindings(AlgaeManipulator algaeManipulator)
    {
        if (commandCodriverGamepad == null)
        {
            return;
        }

        commandCodriverGamepad.rightStick()
            .onTrue(new MoveAlgaeManipulatorWithGamepad(algaeManipulator, codriverGamepad));

        commandCodriverGamepad.x()
            .onTrue(new InstantCommand(() -> algaeManipulator.togglePivotPosition(), algaeManipulator));
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
        commandCodriverGamepad.b()
            .onTrue(new InstantCommand(() -> coralManipulator.ejectL1()));
    }

    private void configureBindings(CoralReceiver coralReceiver)
    {
        if (commandCodriverGamepad == null)
        {
            return;
        }

        commandCodriverGamepad.leftTrigger()
            .onTrue(new InstantCommand(() -> coralReceiver.move()));

    }

    private void configureBindings(Climber climber)
    {
        if (commandCodriverGamepad == null || codriverGamepad == null)
        {
            return;
        }

        commandCodriverGamepad.start().and(commandCodriverGamepad.back())
            .whileTrue(new Climb(climber, ClimberKeys.winchMotorSpeedKey, codriverGamepad));

        commandCodriverGamepad.rightTrigger()
            .onTrue(new InstantCommand(() -> climber.toggleRatchet()));
    }

    private void configureBindings(Elevator elevator)
    {
        if (commandCodriverGamepad == null || codriverGamepad == null)
        {
            return;
        }
        commandCodriverGamepad.leftStick()
            .onTrue(new MoveElevatorWithGamepad(elevator, codriverGamepad));

        commandCodriverGamepad.a()
            .onTrue(new MoveElevatorToCoralPosition(elevator, codriverGamepad, ElevatorKeys.toleranceKey));
    }

    private void configureSmartDashboard()
    {
        driveTrain.ifPresent(this::configureSmartDashboard);
        climber.ifPresent(this::configureSmartDashboard);
        coralManipulator.ifPresent(this::configureSmartDashboard);
        elevator.ifPresent(this::configureSmartDashboard);

        // Configure chooser widgets:
        configureDriveOrientationChooser(driveOrientationChooser);
        configureAutoChooser(autoChooser);
    }

    private void configureSmartDashboard(DriveTrain driveTrain)
    {
        configureDriverHIDChooser(driverHIDChooser);
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
        SmartDashboard.putNumber(CoralManipulatorKeys.leftRollerMotorL1EjectSpeedKey,
            CoralManipulatorConstants.leftRollerMotorL1EjectSpeed);
        SmartDashboard.putNumber(CoralManipulatorKeys.rightRollerMotorL1EjectSpeedKey,
            CoralManipulatorConstants.rightRollerMotorL1EjectSpeed);
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

    private void configureAutoChooser(SendableChooser<Command> autoChooser)
    {
        // Shoot Twice Autos
        autoChooser.addOption("Leave From Empty Side", new PathPlannerAuto("LeaveEmptySide"));
        autoChooser.addOption("Leave From Processor Side", new PathPlannerAuto("LeaveProcessorSide"));

        //Score 
        autoChooser.addOption("Score L4 Empty Side", new PathPlannerAuto("L4ScoreEmptySide"));
        autoChooser.addOption("Score L4 Processor Side", new PathPlannerAuto("L4ScoreProcessorSide"));
        autoChooser.addOption("Score L4 Middle", new PathPlannerAuto("L4ScoreMiddle"));

        SmartDashboard.putData("Auto Selection", autoChooser);
    }

    private void configureDriverHIDChooser(SendableChooser<Command> driverHIDChooser)
    {
        boolean defaultSet = false;
        if (driverGamepad != null)
        {
            driverHIDChooser.setDefaultOption("Drive with Gamepad",
                new DriveWithGamepad(driveTrain.get(), driverGamepad, () ->
                {
                    return isDrivingFieldRelative;
                }));
            defaultSet = true;
        }
        if (driverJoystick != null)
        {
            if (!defaultSet)
            {
                driverHIDChooser.setDefaultOption("Drive with Joystick",
                    new DriveWithJoystick(driveTrain.get(), driverJoystick, () ->
                    {
                        return isDrivingFieldRelative;
                    }));
            }
            else
            {
                driverHIDChooser.addOption("Drive with Joystick",
                    new DriveWithJoystick(driveTrain.get(), driverJoystick, () ->
                    {
                        return isDrivingFieldRelative;
                    }));
            }
        }
        SmartDashboard.putData("Driver Input", driverHIDChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
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

        if (climber.isPresent())
        {
            group.addCommands(new TestClimber(climber.get()));
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

        if (coralReceiver.isPresent())
        {
            group.addCommands(new TestCoralReceiver(coralReceiver.get()));
        }

        if (algaeManipulator.isPresent())
        {
            group.addCommands(new TestAlgaeManipulator(algaeManipulator.get()));
        }

        return group;
    }

    /**
     * Sets configuration related to driving.
     * 
     * This is called from {@link Robot#teleopInit()} so the correct values
     * can be obtained from SmartDashboard choosers.
     */
    void setDriveConfiguration()
    {
        if (driveTrain.isPresent())
        {
            // Set the default command for driving.
            driveTrain.get().setDefaultCommand(driverHIDChooser.getSelected());
        }

        isDrivingFieldRelative = driveOrientationChooser.getSelected();
    }
}
