// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import swervelib.motors.SwerveMotor;

/**
 * Tests the drive motor of a swerve module.
 */
public class TestSwerveModuleDriveMotor extends Command
{
    private final SwerveMotor motor;
    private final double percentOutput;
    private final String moduleName;

    public TestSwerveModuleDriveMotor(DriveTrain driveTrain, String moduleName, double percentOutput)
    {
        this.motor = driveTrain.swerveDrive.getModuleMap().get(moduleName).getDriveMotor();
        this.moduleName = moduleName;
        this.percentOutput = percentOutput;
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        
        System.out.println("testing Drive Motor: " + moduleName  + (percentOutput > 0 ? " forward" : " reverse"));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        motor.set(percentOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        motor.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
