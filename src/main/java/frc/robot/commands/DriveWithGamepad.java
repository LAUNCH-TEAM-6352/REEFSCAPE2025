package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveWithGamepad extends Command
{
    private final DriveTrain DriveTrain;
    private final XboxController driverController;
    private final SendableChooser<Boolean> driveOrientationChooser;

    private Optional<Alliance> alliance;

    public DriveWithGamepad(DriveTrain driveTrain, XboxController driverController,
        SendableChooser<Boolean> driveOrientationChooser)
    {
        this.DriveTrain = driveTrain;
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
        // Determine if we need to invert drive directions based upon drive orientation:
        var isFieldRelative = driveOrientationChooser.getSelected();
        var fieldInversionFactor = isFieldRelative && alliance.isPresent() && alliance.get() == Alliance.Red
            ? -1
            : 1;

        // Get gamePad inputs
        double leftX = driverController.getLeftX();
        double leftY = -driverController.getLeftY();
        double rightX = driverController.getRightX();
        double rightY = -driverController.getRightY();

        // Apply deadband and sensitivity adjustments
        leftX = fieldInversionFactor * applyDeadbandAndSensitivity(leftX);
        leftY = fieldInversionFactor * applyDeadbandAndSensitivity(leftY);
        rightX = applyDeadbandAndSensitivity(rightX);
        rightY = applyDeadbandAndSensitivity(rightY);

        // Drives according to linear speed, rotational speed, and if field is relative (true for now)
        double speedX = leftY * SwerveConstants.maximumLinearVelocityMps;
        double speedY = -leftX * SwerveConstants.maximumLinearVelocityMps;
        double rotationRate = -rightX * SwerveConstants.maximumRotationRateRps;
        Translation2d translationSpeed = new Translation2d(speedX, speedY);
        DriveTrain.drive(translationSpeed, rotationRate, isFieldRelative);
    }

    private double applyDeadbandAndSensitivity(double input)
    {
        return (Math.abs(input) < OperatorConstants.gamepadDeadband) ? 0.0 : Math.pow(input, 2) * Math.signum(input);
    }

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
