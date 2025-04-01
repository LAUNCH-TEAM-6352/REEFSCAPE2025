// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.DashboardConstants.CoralManipulatorKeys;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.CoralManipulator;

/**
 * Tests the CoralManipulator subsystem.
 */
public class TestCoralManipulator extends SequentialCommandGroup
{
    /** Creates a new TestCoralManipulator. */
    public TestCoralManipulator(CoralManipulator coralManipulator)
    {
        addRequirements(coralManipulator);
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Coral Manipulator: Intake")),
            new StartEndCommand(
                () -> coralManipulator.setRollerSpeed(SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorIntakeSpeedKey, CoralManipulatorConstants.rollerMotorIntakeSpeed)),
                () -> coralManipulator.stop()).withTimeout((TestConstants.coralManipulatorTimeoutSecs)),
            new WaitCommand(TestConstants.betweenTimeSecs),

            new InstantCommand(() -> System.out.println("Testing Coral Manipulator: Backup")),
            new StartEndCommand(
                () -> coralManipulator.setRollerSpeed(SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorBackupSpeedKey, CoralManipulatorConstants.rollerMotorBackupSpeed)),
                () -> coralManipulator.stop()).withTimeout((TestConstants.coralManipulatorTimeoutSecs)),
            new WaitCommand(TestConstants.betweenTimeSecs),

            new InstantCommand(() -> System.out.println("Testing Coral Manipulator: Eject")),
            new StartEndCommand(
                () -> coralManipulator.setRollerSpeed(SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorEjectSpeedKey, CoralManipulatorConstants.rollerMotorEjectSpeed)),
                () -> coralManipulator.stop()).withTimeout((TestConstants.coralManipulatorTimeoutSecs)),
            new WaitCommand(TestConstants.betweenTimeSecs),

            new InstantCommand(() -> System.out.println("Testing Coral Manipulator: Eject L1")),
            new StartEndCommand(
                () -> coralManipulator.setRollerSpeed(SmartDashboard.getNumber(CoralManipulatorKeys.leftRollerMotorL1EjectSpeedKey, CoralManipulatorConstants.leftRollerMotorL1EjectSpeed),
                    SmartDashboard.getNumber(CoralManipulatorKeys.rightRollerMotorL1EjectSpeedKey, CoralManipulatorConstants.rightRollerMotorL1EjectSpeed)),
                () -> coralManipulator.stop()).withTimeout((TestConstants.coralManipulatorTimeoutSecs)),
            new WaitCommand(TestConstants.betweenTimeSecs)
        );
    }
}
