// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.PIDConstants;
import frc.robot.Constants.ElevatorConstants.AlternateEncoderConstants;

public class Elevator extends SubsystemBase
{

    private double targetPosition;
    private double targetTolerance;
    private boolean atTargetPosition;
    private boolean isPositioningStarted;
    private double lastPosition;

    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.leftMotorChannel,
        MotorType.kBrushless);

    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.rightMotorChannel,
        MotorType.kBrushless);

    /** Creates a new Elevator. */
    public Elevator()
    {
        AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig()
            .averageDepth(64)
            .countsPerRevolution(8192)
            .inverted(AlternateEncoderConstants.isInverted) // by default, cw rotation is negative
            .measurementPeriod(100)
            .positionConversionFactor(1)
            .setSparkMaxDataPortConfig()
            .velocityConversionFactor(1); // velocity will be measured in hex shaft rotations per minuite

        SoftLimitConfig softLimitConfig = new SoftLimitConfig()
            .forwardSoftLimit(ElevatorConstants.maxPosition)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorConstants.minPosition)
            .reverseSoftLimitEnabled(true);

        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
            .pidf(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD, PIDConstants.kFF)
            .iZone(PIDConstants.kIZ)
            .outputRange(PIDConstants.minOutput, PIDConstants.maxOutput)
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);

        for (SparkMax motor : new SparkMax[] { leftMotor, rightMotor })
        {
            SparkMaxConfig config = new SparkMaxConfig();
            config
                .idleMode(ElevatorConstants.motorIdleMode);

            if (motor == leftMotor)
            {
                config.inverted(ElevatorConstants.isLeftMotorInverted);
                config
                    .apply(encoderConfig)
                    .apply(softLimitConfig)
                    .apply(closedLoopConfig);

            }
            else
            {
                config.follow(leftMotor.getDeviceId(), true);
            }
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            motor.clearFaults();
            resetPosition();
        }

    }

    public void setMotorSpeed(double speed)
    {
        leftMotor.set(speed);
    }

    public double getPosition()
    {
        return leftMotor.getAlternateEncoder().getPosition();
    }

    public void resetPosition()
    {
        leftMotor.getAlternateEncoder().setPosition(0);
    }

    public void setPosition(double position, double tolerance)
    {
        if (position > ElevatorConstants.maxPosition)
        {
            position = ElevatorConstants.maxPosition;
        }
        else if (position < ElevatorConstants.minPosition)
        {
            position = ElevatorConstants.minPosition;
        }

        targetPosition = position;
        targetTolerance = tolerance;
        lastPosition = getPosition();
        leftMotor.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
        atTargetPosition = false;
        isPositioningStarted = true;
    }

    public boolean atTargetPosition()
    {
        return atTargetPosition;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run

        var position = getPosition();

        SmartDashboard.putNumber("Elev Pos", position);
        SmartDashboard.putNumber("Elev Vel", leftMotor.getAlternateEncoder().getVelocity());
        SmartDashboard.putNumber("Elev Spd", rightMotor.getAppliedOutput());

        if (isPositioningStarted)
        {
            if ((Math.abs(position - targetPosition) < targetTolerance)
                && Math.abs(position - lastPosition) < targetTolerance)
            {
                atTargetPosition = true;
                isPositioningStarted = false;
            }
            else
            {
                lastPosition = position;
            }
        }
    }
}
