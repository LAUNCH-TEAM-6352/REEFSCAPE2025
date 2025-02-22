// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorWithGamepad extends Command

{
    private final Elevator elevator;
    private final CommandXboxController gamepad;

    /** Creates a new MoveElevatorWithGamepad. */
    public MoveElevatorWithGamepad(Elevator elevator, CommandXboxController gamepad)
    {
        this.elevator = elevator;
        this.gamepad = gamepad;
        addRequirements(elevator);
    }

    {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        var speed = -gamepad.getRightY();
        var position = elevator.getPosition();
        if ((speed < 0 && position <= ElevatorConstants.minPosition) ||
            (speed > 0 && position >= ElevatorConstants.maxPosition))
        {
            speed = 0;
            gamepad.setRumble(RumbleType.kBothRumble, 1);
        }
        else
        {
            gamepad.setRumble(RumbleType.kBothRumble, 0);
        }
        elevator.setMotorSpeed(speed);
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
