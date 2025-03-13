package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
 * Note the differences between the HID stick and robot coordinate systems:
 * 
 *  Gamepad coordinate system:
 *      - X-axis: - is left, + is right
 *      - Y-axis: - is up, + is down
 *      - Rotation: - is counter-clockwise, + is clockwise
 *      - D-pad: 0 is up, 90 is right, 180 is down, 270 is left
 * 
 *  Robot coordinate system:
 *      - X-axis: + is forward, - is backward
 *      - Y-axis: + is left, - is right
 *      - Rotation: + is counter-clockwise, - is clockwise
 */

public abstract class DriveWithGamepadOrJoystick extends Command
{
    private final DriveTrain driveTrain;
    private final GenericHID driverController;
    private final SendableChooser<Boolean> driveOrientationChooser;

    private Optional<Alliance> alliance;

    public DriveWithGamepadOrJoystick(DriveTrain driveTrain, GenericHID driverController, SendableChooser<Boolean> driveOrientationChooser)
    {
        this.driveTrain = driveTrain;
        this.driverController = driverController;
        this.driveOrientationChooser = driveOrientationChooser;

        // Specify subsystem dependencies (if any)
        addRequirements(driveTrain);
    }

    @Override
    public void initialize()
    {
        alliance = DriverStation.getAlliance();
    }

    @Override
    public void execute()
    {
        // See if we are nudging with the D-pad:
        var translation = DriveConstants.nudgeTranslations.get(driverController.getPOV());
        if (translation != null)
        {
            // Nudging is done robot relative:
            driveTrain.drive(translation, 0.0, false);
            return;
        }

        // Determine if we need to invert drive directions based upon drive orientation:
        var isFieldRelative = driveOrientationChooser.getSelected();
        var fieldInversionFactor = isFieldRelative && alliance.isPresent() && alliance.get() == Alliance.Red
            ? -1
            : 1;

        // Get gamePad stick inputs
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
        driveTrain.drive(translationSpeed, rotationRate, isFieldRelative);
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
