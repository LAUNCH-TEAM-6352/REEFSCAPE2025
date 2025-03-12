// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.CoralReceiver;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestCoralReceiver extends SequentialCommandGroup
{
    
    /** Creates a new TestCoralReceiver. */
    public TestCoralReceiver(CoralReceiver coralReceiver)
    {
        addRequirements(coralReceiver);
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Coral Receiver: Raising")),
            new InstantCommand(() -> coralReceiver.move()),
            new WaitCommand(TestConstants.instantInBetweenSecs),

            new InstantCommand(() -> System.out.println("Testing Coral Receiver: Lowering")),
            new InstantCommand(() -> coralReceiver.move()),
            new WaitCommand(TestConstants.instantInBetweenSecs)
        );
    }

}
