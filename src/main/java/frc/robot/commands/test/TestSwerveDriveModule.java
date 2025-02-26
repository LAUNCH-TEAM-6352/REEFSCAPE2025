// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * Tests a swerve drive moduile.
 */
public class TestSwerveDriveModule extends SequentialCommandGroup
{
    /** Creates a new TestSwerveDriverModule. */
    public TestSwerveDriveModule(DriveTrain driveTrain, String moduleName)
    {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(

        
            new TestSwerveModuleDriveMotor(driveTrain, moduleName, TestConstants.swerveModuleDriveForwardPercentOutput).withTimeout(TestConstants.swerveModuleMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestSwerveModuleDriveMotor(driveTrain, moduleName, TestConstants.swerveModuleDriveReversePercentOutput).withTimeout(TestConstants.swerveModuleMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestSwerveModuleAngleMotor(driveTrain, moduleName, TestConstants.swerveModuleAngleCcwPercentOutput).withTimeout(TestConstants.swerveModuleMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new TestSwerveModuleAngleMotor(driveTrain, moduleName, TestConstants.swerveModuleAngleCwPercentOutput).withTimeout(TestConstants.swerveModuleMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs)
        );
    }
}
