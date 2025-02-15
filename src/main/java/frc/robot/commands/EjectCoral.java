// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectCoral extends Command
{
    private final CoralManipulator coralManipulator;
    private final Elevator elevator;

    /** Creates a new EjectCoral. */
    public EjectCoral(CoralManipulator coralManipulator, Elevator elevator)
    {
        this.coralManipulator = coralManipulator;
        this.elevator = elevator;
        // Will require Coral Manipulator, Elevator
        addRequirements(coralManipulator, elevator);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {

        // Run Coral Manipulator until both beams are not broken
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
