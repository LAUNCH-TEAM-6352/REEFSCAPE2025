// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Climber extends SubsystemBase
{
    private final SparkMax winchMotor = new SparkMax(ClimberConstants.winchMotorChannel, MotorType.kBrushless);

    /** Creates a new Climber. */
  public Climber() 
  {

    SparkMaxConfig config = new SparkMaxConfig();
    config .idleMode (ClimberConstants.motorIdleMode);
    winchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    winchMotor.clearFaults();
  }

  public void setClimbSpeed(double speed)
  {
    winchMotor.set(speed);
  }

  public void stop()
    {
        winchMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber/RPM", winchMotor.getEncoder().getVelocity());
    }
}
