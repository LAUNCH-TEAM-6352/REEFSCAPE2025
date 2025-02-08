// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
    private final Climber climber;
    private double speed;
    private final String speedKey;

  /** Creates a new Climb. */
  public Climb(Climber climber, String speedKey) 
  {
    this.climber = climber;
    //Will also require Coral Reciever
    addRequirements(climber);
    this.speedKey = speedKey;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {   
    //Move Coral Reciever system out of the way
speed = SmartDashboard.getNumber(speedKey, ClimberConstants.winchMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    climber.setClimbSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
