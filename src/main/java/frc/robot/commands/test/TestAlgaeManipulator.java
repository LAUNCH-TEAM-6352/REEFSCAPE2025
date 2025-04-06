// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.AlgaeManipulator;

/**
 * Tests the AlgaeManipulator subsystem.
 */
public class TestAlgaeManipulator extends SequentialCommandGroup
{
    /** Creates a new TestCoralReceiver. */
    public TestAlgaeManipulator(AlgaeManipulator algaeManipulator)
    {
        addRequirements(algaeManipulator);
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Algae Manipulator: Lowering")),
            new InstantCommand(() -> algaeManipulator.togglePivotPosition()),
            new WaitCommand(TestConstants.instantBetweenTimeSecs),

            new InstantCommand(() -> System.out.println("Testing Algae Manipulator: Raising")),
            new InstantCommand(() -> algaeManipulator.togglePivotPosition()),
            new WaitCommand(TestConstants.instantBetweenTimeSecs)
        );
    }
}
