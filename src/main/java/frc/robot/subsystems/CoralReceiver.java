// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralReceiverConstants;
import frc.robot.Constants.ElevatorConstants.PIDConstants;

/**
 * A subsystem for the Coral Receiver tray.
 */
public class CoralReceiver extends SubsystemBase
{
    @SuppressWarnings("unused")
    private double targetPosition;
    @SuppressWarnings("unused")
    private double targetTolerance;
    @SuppressWarnings("unused")
    private boolean atTargetPosition;
    @SuppressWarnings("unused")
    private boolean isPositioningStarted;
    @SuppressWarnings("unused")
    private double lastPosition;

    private final SparkMax motor = new SparkMax(CoralReceiverConstants.motorChannel,
        MotorType.kBrushless);

    /** Creates a new CoralReciever. */
    public CoralReceiver()
    {
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

    private void resetPosition()
    {
        motor.getEncoder().setPosition(0);
    }

    /**
     *  Move the receiver trqy to the up position.
     */
    public void moveUp()
    {

    }

    /**
     * Move the receiver trqy to the down position.
     * 
     * TODO: Do we really need this?
     */
    public void moveDown()
    {

    }

    @Override
    public void periodic()

    {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Receiver Pos", motor.getEncoder().getPosition());
    }
}
