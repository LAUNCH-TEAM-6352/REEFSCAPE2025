// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.ElevatorConstants.PIDConstants;
import frc.robot.subsystems.Elevator;

/**
 * A command that moves the elevator to a pre-defined position based upon gamepad input.
 */
public class MoveElevatorToCoralPosition extends Command
{
    private final Elevator elevator;
    private final XboxController gamepad;
    private final String toleranceKey;
    private CoralLevel targetCoralLevel;

    // Maps from gamepad D-pad values to coral levels.
    // Note the handling of intermediate "headings" assume intended elevator level.
    private final HashMap<Integer, CoralLevel> coralLevels = new HashMap<>(){
        {
            put(-1, CoralLevel.Intake);
            put(0, CoralLevel.Reef2);
            put(45, CoralLevel.Reef2);
            put(90, CoralLevel.Reef3);
            put(135, CoralLevel.Reef4);
            put(180, CoralLevel.Reef4);
            put(225, CoralLevel.Reef4);
            put(270, CoralLevel.Reef1);
            put(315, CoralLevel.Reef2);
        }
    };

    /**
     * Constructor.
     */
    public MoveElevatorToCoralPosition(Elevator elevator, XboxController gamepad, String toleranceKey)
    {
        this.toleranceKey = toleranceKey;
        this.elevator = elevator;
        this.gamepad = gamepad;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        targetCoralLevel = coralLevels.get(gamepad.getPOV());
        var tolerance = SmartDashboard.getNumber(toleranceKey, PIDConstants.tolerance);
        elevator.setPosition(targetCoralLevel.elevatorPosition(), tolerance);
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
        if (targetCoralLevel == CoralLevel.Intake)
        {
            elevator.resetPosition();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return elevator.atTargetPosition();
    }
}
