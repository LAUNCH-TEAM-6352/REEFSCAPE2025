package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * A command that allows the robot to be driven with a joystick.
 * 
 * Note the differences between the joystick and robot coordinate systems:
 * 
 *  Joystick coordinate system:
 *      - Stick X-axis: - is left, + is right
 *      - Stick Y-axis: - is up, + is down
 *      - Rotation: - is counter-clockwise, + is clockwise
 *      - D-pad: 0 is up, 90 is right, 180 is down, 270 is left
 * 
 *  Robot coordinate system:
 *      - X-axis: + is forward, - is backward
 *      - Y-axis: + is left, - is right
 *      - Rotation: + is counter-clockwise, - is clockwise
 */

public class DriveWithJoystick extends DriveWithGamepadOrJoystick
{
    private final Joystick driverController;

    public DriveWithJoystick(DriveTrain driveTrain, Joystick driverController, BooleanSupplier isDrivingFieldRelative)
    {
        super(driveTrain, driverController, isDrivingFieldRelative);
        this.driverController = driverController;
    }

    @Override
    protected double getX()
    {
        return driverController.getX();
    }

    @Override
    protected double getY()
    {
        return driverController.getY();
    }

    @Override
    protected double getRotation()
    {
        return driverController.getTwist();
    }

    @Override
    protected double applyDeadbandAndSensitivity(double input)
    {
        return (Math.abs(input) < OperatorConstants.joystickDeadband) ? 0.0 : Math.pow(input, 2) * Math.signum(input);
    }
}
