// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralReceiverConstants;
import frc.robot.Constants.CoralReceiverConstants.PIDConstants;;

/**
 * A subsystem for the Coral Receiver tray.
 */
public class CoralReceiver extends SubsystemBase
{
    private double targetPosition;
    private double targetTolerance;
    private boolean atTargetPosition;
    private boolean isPositioningStarted;
    private double lastPosition;

    private final SparkMax motor = new SparkMax(CoralReceiverConstants.motorChannel,
        MotorType.kBrushless);

    /** Creates a new CoralReciever. */
    public CoralReceiver()
    {
        targetPosition = CoralReceiverConstants.maxPosition;
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
            .pidf(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD, PIDConstants.kFF)
            .iZone(PIDConstants.kIZ)
            .outputRange(PIDConstants.minOutput, PIDConstants.maxOutput);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .apply(closedLoopConfig)
            .idleMode(CoralReceiverConstants.motorIdleMode)
            .inverted(CoralReceiverConstants.isMotorInverted);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.clearFaults();

        resetPosition();
    }

    public double getPosition()
    {
        return motor.getEncoder().getPosition();
    }

    private void resetPosition()
    {
        motor.getEncoder().setPosition(0);
    }

    /**
     *  Move the receiver tray to the up position.
     */
    public void moveUp()
    {
        targetTolerance = CoralReceiverConstants.PIDConstants.tolerance;
        lastPosition = getPosition();
        motor.getClosedLoopController().setReference(CoralReceiverConstants.maxPosition, ControlType.kPosition);
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
        var position = getPosition();
        
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Receiver Pos", position);

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
