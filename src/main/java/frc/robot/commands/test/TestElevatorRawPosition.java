// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.CoralLevel;
import frc.robot.Constants.ElevatorConstants.PIDConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestElevatorRawPosition extends Command
{

    private final Elevator elevator;
    private final double position;
    private final String toleranceKey;

    /** Creates a new TestElevatorPosition. */
    public TestElevatorRawPosition(Elevator elevator, double position, String toleranceKey)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.position = position;
        this.elevator = elevator;
        this.toleranceKey = toleranceKey;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        var tolerance = SmartDashboard.getNumber(toleranceKey, PIDConstants.tolerance);
        System.out.println("Running Elevator Position: " +  position);
        elevator.setPosition(position, tolerance);
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
        return elevator.atTargetPosition();
    }
}
