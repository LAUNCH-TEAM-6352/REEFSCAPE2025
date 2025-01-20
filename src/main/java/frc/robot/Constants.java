// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    public static class DriveConstants
    {
        public static final TelemetryVerbosity swerveDriveTelemetryVerbosity = TelemetryVerbosity.HIGH;
    }

    //TODO: Confirm values
    public static class OperatorConstants
    {
        public static final int driverGamepadPort = 1;
        public static final int codriverGamepadPort = 2;
        public static final double gamepadDeadband = 0.05;

    }
    //TODO: Confirm values
    public static final class PathPlannerConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }


    //TODO: Confirm values
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
