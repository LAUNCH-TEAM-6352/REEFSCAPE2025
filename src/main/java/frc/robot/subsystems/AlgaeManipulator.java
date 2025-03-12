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
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.PIDConstants;
import frc.robot.Constants.CoralReceiverConstants;

/**
 * A subsystem for the Algae Manipulator.
 */
public class AlgaeManipulator extends SubsystemBase
{
    private double targetPosition;
    private double targetTolerance;
    private boolean atTargetPosition;
    private boolean isPositioningStarted;
    private double lastPosition;
    private double currentPosition;

    private final SparkMax pivotMotor = new SparkMax(AlgaeConstants.pivotMotorChannel, MotorType.kBrushless);
    private final SparkMax rollerMotor = new SparkMax(AlgaeConstants.rollerMotorChannel, MotorType.kBrushless);

    /** Creates a new AlgaeManipulator. */
    public AlgaeManipulator()
    {
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
            .pidf(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD, PIDConstants.kFF)
            .iZone(PIDConstants.kIZ)
            .outputRange(PIDConstants.minOutput, PIDConstants.maxOutput);
            
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig
            .idleMode(AlgaeConstants.pivotMotorIdleMode)
            .inverted(AlgaeConstants.isPivotMotorInverted)
            .apply(closedLoopConfig);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.clearFaults();
        pivotMotor.getEncoder().setPosition(AlgaeConstants.storedPivotPosition);

        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig
            .idleMode(AlgaeConstants.rollerMotorIdleMode)
            .inverted(AlgaeConstants.isRollerMotorInverted);

        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.clearFaults();

        resetPivotPosition();
        currentPosition = AlgaeConstants.storedPivotPosition;
    }

    public double getPivotPosition()
    {
        return pivotMotor.getEncoder().getPosition();
    }

    public void togglePivotPosition()
    {
        targetTolerance = AlgaeConstants.PIDConstants.tolerance;
        lastPosition = getPivotPosition();
        targetPosition = currentPosition == AlgaeConstants.storedPivotPosition
            ? AlgaeConstants.activePivotPosition
            : AlgaeConstants.storedPivotPosition;
        pivotMotor.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
        atTargetPosition = false;
        isPositioningStarted = true;
    }

    public boolean atTargetPosition()
    {
        return atTargetPosition;
    }

    public void resetPivotPosition()
    {
        pivotMotor.getEncoder().setPosition(0);
    }

    public void setPivotMotorSpeed(double speed)
    {
        pivotMotor.set(speed);
    }

    public void setRollerMotorSpeed(double speed)
    {
        rollerMotor.set(speed);
    }

    @Override
    public void periodic()
    {
        var position = getPivotPosition();

        SmartDashboard.putNumber("Pivot Pos", pivotMotor.getEncoder().getPosition());

        if (isPositioningStarted)
        {
            if ((Math.abs(position - targetPosition) < targetTolerance)
                && Math.abs(position - lastPosition) < targetTolerance)
            {
                atTargetPosition = true;
                isPositioningStarted = false;
                currentPosition = currentPosition == AlgaeConstants.storedPivotPosition
                    ? AlgaeConstants.activePivotPosition
                    : AlgaeConstants.storedPivotPosition;
            }
            else
            {
                lastPosition = position;
            }
        }
    }
}
