// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestClimber extends SequentialCommandGroup
{

    /** Creates a new TestClimber. */
    public TestClimber(Climber climber)
    {
        addRequirements(climber);
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Climber Ratchet: Releasing")),
            new InstantCommand(() -> climber.toggleRatchet()),
            new WaitCommand(TestConstants.instantInBetweenSecs),

            new InstantCommand(() -> System.out.println("Testing Climber Ratchet: Engaging")),
            new InstantCommand(() -> climber.toggleRatchet()),
            new WaitCommand(TestConstants.instantInBetweenSecs)
        );
    }
}
