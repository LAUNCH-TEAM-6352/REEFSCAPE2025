package frc.robot.commands;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * An abstract command that is used for driving the robot using a gamepad or
 * joystick. This command is intended to be extended by other commands that
 * explicitly support a gamepad or joystick. This abstract implementation
 * assumes thast the gamepad and joystick have similar stick coordinate systems.
 * 
 * Note the differences between the HID stick, robot and field coordinate systems:
 * 
 *  Gamepad/Joystick coordinate system:
 *      - X-axis: - is left, + is right
 *      - Y-axis: - is up, + is down
 *      - Rotation: - is counter-clockwise, + is clockwise
 *      - D-pad: 0 is up, 90 is right, 180 is down, 270 is left
 * 
 *  Robot coordinate system:
 *      - X-axis: + is forward, - is backward
 *      - Y-axis: + is left, - is right
 *      - Rotation: + is counter-clockwise, - is clockwise
 *  
 *  Field coordinate system (when looking at field from above with blue alliance on the left):
 *     - Origin: bottom left corner of the field (blue alliance corner)
 *     - X-axis: + is right, - is left
 *     - Y-axis: + is up, - is down
 *     - Rotation: + is counter-clockwise, - is clockwise
 */

public abstract class DriveWithGamepadOrJoystick extends Command
{
    private final DriveTrain driveTrain;
    private final GenericHID driverController;
    private final BooleanSupplier isDrivingFieldRelativeSupplier;

    private boolean isDrivingFieldRelative;

    private Optional<Alliance> alliance;


    // Maps from joystick/gamepad D-pad values to robot-oriented translation speeds for nudging the robot:
    private static final HashMap<Integer, Translation2d> nudgeTranslations = new HashMap<>()
    {
        {
            put(0, new Translation2d(DriveConstants.nudgeSpeedMps, 0.0));
            put(45, new Translation2d(DriveConstants.nudgeSpeedMps, -DriveConstants.nudgeSpeedMps));
            put(90, new Translation2d(0.0, -DriveConstants.nudgeSpeedMps));
            put(135, new Translation2d(-DriveConstants.nudgeSpeedMps, -DriveConstants.nudgeSpeedMps));
            put(180, new Translation2d(-DriveConstants.nudgeSpeedMps, 0.0));
            put(225, new Translation2d(-DriveConstants.nudgeSpeedMps, DriveConstants.nudgeSpeedMps));
            put(270, new Translation2d(0.0, DriveConstants.nudgeSpeedMps));
            put(315, new Translation2d(DriveConstants.nudgeSpeedMps, DriveConstants.nudgeSpeedMps));
        }
    };

    /**
     * Constructor for the DriveWithGamepadOrJoystick command.
     */
    public DriveWithGamepadOrJoystick(DriveTrain driveTrain, GenericHID driverController, BooleanSupplier isDrivingFieldRelativeSupplier)
    {
        this.driveTrain = driveTrain;
        this.driverController = driverController;
        this.isDrivingFieldRelativeSupplier = isDrivingFieldRelativeSupplier;

        // Specify subsystem dependencies (if any)
        addRequirements(driveTrain);
    }

    @Override
    public void initialize()
    {
        alliance = DriverStation.getAlliance();
        isDrivingFieldRelative = isDrivingFieldRelativeSupplier.getAsBoolean();
    }

    @Override
    public void execute()
    {
        // See if we are nudging with the D-pad:
        var translation = nudgeTranslations.get(driverController.getPOV());
        if (translation != null)
        {
            // Nudging is done robot relative:
            driveTrain.drive(translation, 0.0, false);
            return;
        }

        // If we are drivcing field relative, we need to invert X and Y if our alliance is red.
        // This is because the field's coordinate system has its origin in the blue alliance corner
        var fieldInversionFactor = isDrivingFieldRelative && alliance.isPresent() && alliance.get() == Alliance.Red
            ? -1
            : 1;

        // Get gamepad/joystick inputs
        double stickX = getX();
        double stickY = -getY();
        double rotation = getRotation();

        // Apply deadband and sensitivity adjustments
        stickX = fieldInversionFactor * applyDeadbandAndSensitivity(stickX);
        stickY = fieldInversionFactor * applyDeadbandAndSensitivity(stickY);
        rotation = applyDeadbandAndSensitivity(rotation);

        // Drives according to linear speed, rotational speed, and if field is relative
        double speedX = stickY * SwerveConstants.maximumLinearVelocityMps;
        double speedY = -stickX * SwerveConstants.maximumLinearVelocityMps;
        double rotationRate = -rotation * SwerveConstants.maximumRotationRateRps;
        Translation2d translationSpeed = new Translation2d(speedX, speedY);
        driveTrain.drive(translationSpeed, rotationRate, isDrivingFieldRelative);
    }

    // Abstract method to get the X-axis value from the gamepad or joystick
    protected abstract double getX();

    // Abstract method to get the Y-axis value from the gamepad or joystick
    protected abstract double getY();

    // Abstract method to get the twist value from the gamepad or joystick
    protected abstract double getRotation();

    // Abstract method to apply deadband and sensitivity adjustments to the input
    protected abstract double applyDeadbandAndSensitivity(double input);


    @Override
    public void end(boolean interrupted)
    {
        // Perform any actions when the command ends
    }

    @Override
    public boolean isFinished()
    {
        // This command is intended to run continuously during teleop
        return false;
    }
}
