// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DashboardConstants.ElevatorKeys;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.CoralLevel;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestElevator extends SequentialCommandGroup
{
    /** Creates a new TestElevator. */
    public TestElevator(Elevator elevator)
    {
        addCommands(
            new TestElevatorCoralPosition(elevator, CoralLevel.Intake, ElevatorKeys.toleranceKey),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestElevatorCoralPosition(elevator, CoralLevel.Reef1, ElevatorKeys.toleranceKey),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestElevatorCoralPosition(elevator, CoralLevel.Reef2, ElevatorKeys.toleranceKey),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestElevatorCoralPosition(elevator, CoralLevel.Reef3, ElevatorKeys.toleranceKey),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestElevatorCoralPosition(elevator, CoralLevel.Reef4, ElevatorKeys.toleranceKey),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestElevatorRawPosition(elevator, ElevatorConstants.minPosition, ElevatorKeys.toleranceKey),
            new WaitCommand(TestConstants.inbetweenTimeSecs)
        );
    }
};