// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAlgaeManipulatorWithGamepad extends Command
{
    private final AlgaeManipulator algaeManipulator;
    private final XboxController gamepad;

    /** Creates a new MoveAlgaeManipulatorWithGamepad. */
    public MoveAlgaeManipulatorWithGamepad(AlgaeManipulator algaeManipulator, XboxController gamepad)
    {
        this.algaeManipulator = algaeManipulator;
        this.gamepad = gamepad;
        addRequirements(algaeManipulator);
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
        var speed = gamepad.getLeftY() * AlgaeConstants.pivotMotorMaxSpeed;
        var position = algaeManipulator.getPivotPosition();
        if ((speed < 0 && position <= AlgaeConstants.storedPivotPosition) ||
            (speed > 0 && position >= AlgaeConstants.activePivotPosition))
        {
            speed = 0;
            gamepad.setRumble(RumbleType.kBothRumble, 1);
        }
        else
        {
            gamepad.setRumble(RumbleType.kBothRumble, 0);
        }
        algaeManipulator.setPivotMotorSpeed(speed);
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
