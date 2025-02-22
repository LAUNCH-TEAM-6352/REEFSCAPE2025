// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.subsystems.CoralManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestCoralManipulator extends Command
{
    private final CoralManipulator coralManipulator;
    private final String speedKey;
    private double speed;

    /** Creates a new TestCoralManipulator. */
    public TestCoralManipulator(CoralManipulator coralManipulator, String speedKey)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralManipulator = coralManipulator;
        this.speedKey = speedKey;
        addRequirements(coralManipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        speed = SmartDashboard.getNumber(speedKey, CoralManipulatorConstants.rollerMotorSpeed);
        System.out.println("Testing Coral Manipulator");
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
