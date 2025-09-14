// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.CoralReceiverConstants.ReceiverPosition;
import frc.robot.Constants.ElevatorConstants.PIDConstants;
import frc.robot.subsystems.CoralReceiver;
import frc.robot.subsystems.Elevator;

/**
 * A command that moves the elevator to a pre-defined position based upon gamepad input.
 */
public class MoveCoralReceiver extends Command
{
    private final CoralReceiver coralReceiver;
    private final XboxController gamepad;
    private double motorSpeed;
    private ReceiverPosition targetReceiverPosition;

    // Maps from gamepad D-pad values to coral levels.
    // Note the handling of intermediate "headings" assume intended elevator level.
    private final HashMap<Integer, ReceiverPosition> receiverPositions = new HashMap<>(){
        {
            put(-1, ReceiverPosition.Current);
            put(0, ReceiverPosition.Up);
            put(45, ReceiverPosition.Up);
            put(90, ReceiverPosition.Current);
            put(135, ReceiverPosition.Down);
            put(180, ReceiverPosition.Down);
            put(225, ReceiverPosition.Down);
            put(270, ReceiverPosition.Current);
            put(315, ReceiverPosition.Up);
        }
    };

    /**
     * Constructor.
     */
    public MoveCoralReceiver(CoralReceiver coralReceiver, XboxController gamepad)
    {
        this.coralReceiver = coralReceiver;
        this.gamepad = gamepad;
        addRequirements(coralReceiver);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        targetReceiverPosition = receiverPositions.get(gamepad.getPOV());
        motorSpeed = targetReceiverPosition == ReceiverPosition.Current
            ? 0
            : SmartDashboard.getNumber(targetReceiverPosition.receiverPositionKey(), 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        coralReceiver.setMotorSpeed(motorSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        coralReceiver.stopMotor();
        if (targetReceiverPosition == ReceiverPosition.Down)
        {
            coralReceiver.resetPosition();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return motorSpeed == 0;
    }
}
