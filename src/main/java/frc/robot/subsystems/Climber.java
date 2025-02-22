// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.EncoderConstants;

public class Climber extends SubsystemBase
{
    private final SparkMax winchMotor = new SparkMax(ClimberConstants.winchMotorChannel, MotorType.kBrushed);

    /** Creates a new Climber. */
    public Climber()
    {
        EncoderConfig encoderConfig = new EncoderConfig()
            .quadratureAverageDepth(64)
            .countsPerRevolution(EncoderConstants.countsPerRevolution)
            .inverted(EncoderConstants.isInverted) 
            .quadratureMeasurementPeriod(100)
            .positionConversionFactor(EncoderConstants.positionConversionFactor)
            .velocityConversionFactor(EncoderConstants.velocityConversionFactor);

        SoftLimitConfig softLimitConfig = new SoftLimitConfig()
            .forwardSoftLimit(ClimberConstants.maxPosition)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ClimberConstants.minPosition)
            .reverseSoftLimitEnabled(true);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(ClimberConstants.motorIdleMode)
            .inverted(ClimberConstants.isMotorInverted)
            .apply(encoderConfig)
            .apply(softLimitConfig);
            
        winchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchMotor.clearFaults();

        winchMotor.getEncoder().setPosition(0);
    }

    public void setClimbSpeed(double speed)
    {
        winchMotor.set(speed);
    }

    public double getPosition()
    {
        return winchMotor.getEncoder().getPosition();
    }

    public void stop()
    {
        winchMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber RPM", winchMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Climber Pos", winchMotor.getEncoder().getPosition());        
    }
}
