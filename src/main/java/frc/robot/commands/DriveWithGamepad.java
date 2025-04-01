package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DashboardConstants.DriveKeys;
import frc.robot.subsystems.DriveTrain;

/**
 * A command that allows the robot to be driven with a gamepad.
 * 
 * Note the differences between the gamepad stick and robot coordinate systems:
 * 
 *  Gamepad coordinate system:
 *      - Stick X-axis: - is left, + is right
 *      - Stick Y-axis: - is up, + is down
 *      - D-pad: 0 is up, 90 is right, 180 is down, 270 is left
 * 
 *  Robot coordinate system:
 *      - X-axis: + is forward, - is backward
 *      - Y-axis: + is left, - is right
 *      - Rotation: + is counter-clockwise, - is clockwise
 */

public class DriveWithGamepad extends DriveWithGamepadOrJoystick
{
    private final XboxController driverController;

    public DriveWithGamepad(DriveTrain driveTrain, XboxController driverController, BooleanSupplier isDrivingFieldRelativeSupplier)
    {
        super(driveTrain, driverController, isDrivingFieldRelativeSupplier, DriveKeys.nudgeSpeedKey);

        this.driverController = driverController;
    }

    @Override
    protected double getX()
    {
        return driverController.getLeftX();
    }

    @Override
    protected double getY()
    {
        return driverController.getLeftY();
    }

    @Override
    protected double getRotation()
    {
        return driverController.getRightX();
    }

    @Override
    protected double applyDeadbandAndSensitivity(double input)
    {
        return (Math.abs(input) < OperatorConstants.gamepadDeadband) ? 0.0 : Math.pow(input, 2) * Math.signum(input);
    }
}
