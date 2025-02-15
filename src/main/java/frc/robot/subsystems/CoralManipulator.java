// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.CoralManipulatorConstants;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase
{

    private final SparkMax leftRollerMotor = new SparkMax(CoralManipulatorConstants.leftRollerMotorChannel,
        MotorType.kBrushless);

    private final SparkMax rightRollerMotor = new SparkMax(CoralManipulatorConstants.rightRollerMotorChannel,
        MotorType.kBrushless);

    private final AnalogInput EntranceCoralSensor = new AnalogInput(
        CoralManipulatorConstants.entranceOpticalSensorPort);

    private final AnalogInput ExitCoralSensor = new AnalogInput(CoralManipulatorConstants.exitOpticalSensorPort);

    /** Creates a new CoralManipulator. */
    public CoralManipulator()
    {
        // Apply configuration common to both roller motors:
        for (SparkMax motor : new SparkMax[] { leftRollerMotor, rightRollerMotor })
        {
            SparkMaxConfig config = new SparkMaxConfig();

            config
                .idleMode(CoralManipulatorConstants.motorIdleMode);

            if (motor == leftRollerMotor)
            {
                config.inverted(CoralManipulatorConstants.isLeftRollerMotorInverted);
            }
            else
            {
                config.follow(CoralManipulatorConstants.leftRollerMotorChannel, true);
            }

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            motor.clearFaults();
        }
    }
    public void setRollerSpeed(double speed)
    {
        leftRollerMotor.set(speed);
    }
    public void stop()
    {
        leftRollerMotor.stopMotor();
    }

    public boolean isEntranceBlocked()
    {
    var voltage = EntranceCoralSensor.getVoltage();
    return voltage > CoralManipulatorConstants.opticalSensorVoltageThreshold;
    }
    public boolean isExitBlocked()
    {
    var voltage = ExitCoralSensor.getVoltage();
    return voltage > CoralManipulatorConstants.opticalSensorVoltageThreshold;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Roller RPM", leftRollerMotor.getEncoder().getVelocity());
    SmartDashboard.putBoolean( "Coral Entrance", !isEntranceBlocked());
    SmartDashboard.putBoolean( "Coral Exit", !isExitBlocked());
    }
}
