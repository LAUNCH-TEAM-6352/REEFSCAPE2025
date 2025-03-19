// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.DashboardConstants.CoralManipulatorKeys;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsytem for receiving and ejecting coral.
 */
public class CoralManipulator extends SubsystemBase
{
    private enum RollerState
    {
        IDLE, INTAKE, BACKUP, EJECT, EXTRA_TIME
    }

    private RollerState rollerState = RollerState.IDLE;

    private double extraTimeSecs;

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

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            motor.clearFaults();
        }
    }

    public void setRollerSpeed(double speed)
    {
        setRollerSpeed(speed, speed);
    }

    public void setRollerSpeed (double leftSpeed, double rightSpeed)
    {
        leftRollerMotor.set(leftSpeed);
        rightRollerMotor.set(rightSpeed);
    }

    public void stop()
    {
        setRollerSpeed(0);
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

    private boolean isCoralBackedUp()
    {
        return isEntranceBlocked() && isExitBlocked();
    }

    private boolean isCoralPartiallyOut()
    {
        return !isEntranceBlocked() && !isExitBlocked();
    }

    public void intakeCoral()
    {
        rollerState = RollerState.INTAKE;
        setRollerSpeed(SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorIntakeSpeedKey, CoralManipulatorConstants.rollerMotorIntakeSpeed));
    }

    public void ejectCoral()
    {
        rollerState = RollerState.EJECT;
        setRollerSpeed(SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorEjectSpeedKey, CoralManipulatorConstants.rollerMotorEjectSpeed));
        extraTimeSecs = SmartDashboard.getNumber(CoralManipulatorKeys.extraTimeSecsKey, CoralManipulatorConstants.extraTimeSecs);
    }

    public void ejectL1()
    {
    
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Coral RPM", leftRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Coral Spd", leftRollerMotor.getAppliedOutput());
        SmartDashboard.putBoolean("Coral Entrance", !isEntranceBlocked());
        SmartDashboard.putBoolean("Coral Exit", !isExitBlocked());

        switch (rollerState)
        {
            case INTAKE:
                if (isCoralIn())
                {
                    setRollerSpeed(SmartDashboard.getNumber(CoralManipulatorKeys.rollerMotorBackupSpeedKey, CoralManipulatorConstants.rollerMotorBackupSpeed));
                    rollerState = RollerState.BACKUP;
                }
                break;

            case BACKUP:
                if (isCoralBackedUp())
                {
                    stop();
                    rollerState = RollerState.IDLE;
                }
                break;

            case EJECT:
                if (isCoralPartiallyOut())
                {
                    stopExtraTime = RobotController.getFPGATime() + (long) (extraTimeSecs * 1e6);
                    rollerState = RollerState.EXTRA_TIME;
                }
                break;

            case EXTRA_TIME:
                if (RobotController.getFPGATime() >= stopExtraTime)
                {
                    stop();
                    rollerState = RollerState.IDLE;
                }

            default:
                break;
        }
    }
}
