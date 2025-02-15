// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Enumeration;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    public static class DriveConstants
    {
        public static final TelemetryVerbosity swerveDriveTelemetryVerbosity = TelemetryVerbosity.HIGH;
    }

    // TODO: Confirm values
    public static class ClimberConstants
    {
        public static final int winchMotorChannel = 0;
        public static final IdleMode motorIdleMode = IdleMode.kBrake;
        public static final double winchMotorSpeed = 0;

    }

    // TODO: Confirm values
    public static class CoralManipulatorConstants
    {
        public static final int leftRollerMotorChannel = 0;
        public static final int rightRollerMotorChannel = 0;
        public static final IdleMode motorIdleMode = IdleMode.kBrake;
        public static final double rollerMotorSpeed = 0;
        public static final int entranceOpticalSensorPort = 0;
        public static final int exitOpticalSensorPort = 0;
        public static final boolean isLeftRollerMotorInverted = false;
        public static final boolean isRightRollerMotorInverted = false;
        public static final double opticalSensorVoltageThreshold = 0.8;

    }

    public static final class DashboardConstants
    {
        public static final class ClimberKeys
        {
            public static final String winchMotorSpeedKey = "Climber Motor Speed";

        }
        public static final class CoralManipulatorKeys
        {
            public static final String rollerMotorSpeedKey = "Coral Manipulator Motor Speed";
            public static final String entranceOpticalSensorKey = "Entrance Optical Sensor";
            public static final String exitOpticalSensorKey = "Exit Optical Sensor";

        }
    }

    // TODO: Confirm values
    public static final class ElevatorConstants
    {
        public static final int elevatorMotorChannel = 0;
        public static final IdleMode motorIdleMode = IdleMode.kBrake;
        public static final double elevatorMotorSpeed = 0;

        public enum CoralLevel
        {
            Intake(0),
            Reef1(1),
            Reef2(2),
            Reef3(3),
            Reef4(4);

            private final int elevatorPosition;

            CoralLevel(int elevatorPosition)
            {
                this.elevatorPosition = elevatorPosition;
            }

            public int elevatorPosition()
            {
                return elevatorPosition;
            }
        }
    }

    // TODO: Confirm values
    public static class OperatorConstants
    {
        public static final int driverGamepadPort = 1;
        public static final int codriverGamepadPort = 2;
        public static final double gamepadDeadband = 0.05;

    }

    // TODO: Confirm values
    public static final class PathPlannerConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }

    // TODO: Confirm values
    public static final class SwerveConstants
    {
        public static final double maximumLinearVelocityMps = 10.0;
        public static final double maximumRotationRateRps = 4 * Math.PI;

        // Don't mess with this!
        public static final double maxModuleSpeedMps = 4.57;
    }

    public static class TestConstants
    {
        public static final double inbetweenTimeSecs = 1;

        public static final double swerveModuleMotorTimeoutSecs = 5;
        public static final double swerveModuleDriveForwardPercentOutput = 0.25;
        public static final double swerveModuleDriveReversePercentOutput = -0.25;
        public static final double swerveModuleAngleCcwPercentOutput = 0.25;
        public static final double swerveModuleAngleCwPercentOutput = -0.25;
    }

}
