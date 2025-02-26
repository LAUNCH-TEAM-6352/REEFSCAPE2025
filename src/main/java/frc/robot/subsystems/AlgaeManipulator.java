// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeManipulator extends SubsystemBase
{
    private final SparkMax pivotMotor = new SparkMax(AlgaeConstants.pivotMotorChannel, MotorType.kBrushless);
    private final SparkMax rollerMotor = new SparkMax(AlgaeConstants.rollerMotorChannel, MotorType.kBrushless);

    
    /** Creates a new AlgaeManipulator. */
    public AlgaeManipulator()
    {
        // TODO: Do we need to add a PID controller and support for moving to preset positions?
        SoftLimitConfig softLimitConfig = new SoftLimitConfig()
            .forwardSoftLimit(AlgaeConstants.pivotMotorMaxPosition)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(AlgaeConstants.pivotMotorMinPosition)
            .reverseSoftLimitEnabled(true);
            
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig
            .idleMode(AlgaeConstants.pivotMotorIdleMode)
            .inverted(AlgaeConstants.isPivotMotorInverted)
            .apply(softLimitConfig);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.clearFaults();
        pivotMotor.getEncoder().setPosition(0);

        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig
            .idleMode(AlgaeConstants.rollerMotorIdleMode)
            .inverted(AlgaeConstants.isRollerMotorInverted);

        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.clearFaults();
    }

    public void setPivotMotorSpeed(double speed)
    {
        pivotMotor.set(speed);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Pivot Pos", pivotMotor.getEncoder().getPosition());
    }
}
