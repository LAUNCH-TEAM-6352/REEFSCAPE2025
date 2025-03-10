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
    private double speed;

    /** Creates a new TestCoralManipulator. */
    public TestCoralManipulator(CoralManipulator coralManipulator, double speed)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralManipulator = coralManipulator;
        this.speed = speed;
        addRequirements(coralManipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        System.out.println("Testing Coral Manipulator: " + speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        coralManipulator.setRollerSpeed(speed);
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
