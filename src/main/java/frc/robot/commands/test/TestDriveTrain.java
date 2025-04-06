// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * Tests the DriveTrain subsystem.
 */
public class TestDriveTrain extends SequentialCommandGroup
{
    public TestDriveTrain(DriveTrain driveTrain)
    {
        addCommands(
            new TestSwerveDriveModule(driveTrain, "backleft"),
            new WaitCommand(TestConstants.betweenTimeSecs),

            new TestSwerveDriveModule(driveTrain, "backright"),
            new WaitCommand(TestConstants.betweenTimeSecs),

            new TestSwerveDriveModule(driveTrain, "frontright"),
            new WaitCommand(TestConstants.betweenTimeSecs),

            new TestSwerveDriveModule(driveTrain, "frontleft"),
            new WaitCommand(TestConstants.betweenTimeSecs)
        );
    }
}
