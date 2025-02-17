// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.CoralManipulatorConstants;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase

{
    private double extraTimeSecs;
    private boolean isIntaking = false;
    private boolean isEjecting = false;
    private boolean inExtraTime = false;
    private long stopExtraTime;

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
                config.follow(leftRollerMotor.getDeviceId(), true);
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

    private boolean isEntranceBlocked()
    {
        var voltage = EntranceCoralSensor.getVoltage();
        return voltage > CoralManipulatorConstants.opticalSensorVoltageThreshold;
    }

    private boolean isExitBlocked()
    {
        var voltage = ExitCoralSensor.getVoltage();
        return voltage > CoralManipulatorConstants.opticalSensorVoltageThreshold;
    }

    private boolean isCoralIn()
    {
        return !isEntranceBlocked() && isExitBlocked();
    }

    private boolean isCoralPartiallyOut()
    {
        return !isEntranceBlocked() && !isExitBlocked();
    }

    public void intakeCoral(String speedKey)
    {
        isIntaking = true;
        setRollerSpeed(SmartDashboard.getNumber(speedKey, CoralManipulatorConstants.rollerMotorSpeed));

    }

    public void ejectCoral(String speedKey, String extraTimeKey)
    {
        isEjecting = true;
        inExtraTime = false;
        setRollerSpeed(SmartDashboard.getNumber(speedKey, CoralManipulatorConstants.rollerMotorSpeed));
        extraTimeSecs = SmartDashboard.getNumber(extraTimeKey, CoralManipulatorConstants.extraTimeSecs);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Coral Roller RPM", leftRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Coral Entrance", !isEntranceBlocked());
        SmartDashboard.putBoolean("Coral Exit", !isExitBlocked());

        if (isIntaking)
        {
            if (isCoralIn())
            {
                stop();
                isIntaking = false;
            }
        }
        else if (isEjecting)
        {
            if (isCoralPartiallyOut())
            {
                if (!inExtraTime)
                {
                    inExtraTime = true;
                    stopExtraTime = RobotController.getFPGATime() + (long) (extraTimeSecs * 1e6);
                }
                else if (RobotController.getFPGATime() >= stopExtraTime)
                {
                    stop();
                    isEjecting = false;
                    inExtraTime = false;
                }
            }
        }
    }
}
