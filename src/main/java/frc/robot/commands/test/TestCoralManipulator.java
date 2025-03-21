// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulator;

/**
 * Tests the CoralManipulator subsystem.
 */
public class TestCoralManipulator extends Command
{
    private final CoralManipulator coralManipulator;
    private Double speed = null;
    private Double leftSpeed = null;
    private Double rightSpeed = null;

    /** Creates a new TestCoralManipulator. */
    public TestCoralManipulator(CoralManipulator coralManipulator, double speed)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralManipulator = coralManipulator;
        this.speed = speed;
        addRequirements(coralManipulator);
    }

    public TestCoralManipulator(CoralManipulator coralManipulator, double leftSpeed, double rightSpeed)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralManipulator = coralManipulator;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        addRequirements(coralManipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (speed != null)
        {
            System.out.println("Testing Coral Manipulator: " + speed);
        }
        else
        {
            System.out.println("Testing Coral Manipulator: " + leftSpeed + ", " + rightSpeed);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (speed != null)
        {
            coralManipulator.setRollerSpeed(speed);
        }
        else
        {
            coralManipulator.setRollerSpeed(leftSpeed, rightSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        coralManipulator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
