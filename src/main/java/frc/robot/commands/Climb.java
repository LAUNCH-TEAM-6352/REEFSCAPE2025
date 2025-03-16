// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

/*
 * A command for climbing.
 * 
 * This command assumes that the coral receiover tray has been moved out of the way.
 */
public class Climb extends Command
{
    private final Climber climber;
    private double speed;
    private final String speedKey;
    private final XboxController gamepad;

    /** Creates a new Climb. */
    public Climb(Climber climber, String speedKey, XboxController gamepad)
    {
        this.gamepad = gamepad;
        this.climber = climber;
        this.speedKey = speedKey;
        // Will also require Coral Reciever
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        speed = SmartDashboard.getNumber(speedKey, ClimberConstants.winchMotorSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        var speed = this.speed;
        var position = climber.getPosition();
        if ((speed < 0 && position <= ClimberConstants.minPosition) ||
            (speed > 0 && position >= ClimberConstants.maxPosition))
        {
            speed = 0;
            gamepad.setRumble(RumbleType.kBothRumble, 1);
        }
        else
        {
            gamepad.setRumble(RumbleType.kBothRumble, 0);
        }
        climber.setClimbSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        climber.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
