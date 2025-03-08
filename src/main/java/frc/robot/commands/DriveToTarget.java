package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightSubsystem;

/** A command that uses Limelight to drive to a target using swerve drive. */
public class DriveToTarget extends Command
{
    private final LimelightSubsystem limelight;
    private final DriveTrain driveTrain;

    private final HolonomicDriveController holonomicController;

    public DriveToTarget(LimelightSubsystem limelight, DriveTrain driveTrain)
    {
        this.limelight = limelight;
        this.driveTrain = driveTrain;
        addRequirements(limelight, driveTrain);
        // aimPIDController = new PIDController(0.08, 0.035, 0.025);
        // rangePIDController = new PIDController(0.15, 0.05, 0.4);

        holonomicController = new HolonomicDriveController(
            new PIDController(1.0, 0.0, 0.0), // X PID
            new PIDController(1.0, 0.0, 0.0), // Y PID
            new ProfiledPIDController(1.0, 0.0, 0.0,
                new TrapezoidProfile.Constraints(DriveConstants.maxSpeedMps, DriveConstants.maxAccelerationMps)) // Theta
                                                                                                                 // PID
        );
    }

    @Override
    public void initialize()
    {
        // var tolerance = SmartDashboard.getNumber(toleranceKey, PIDConstants.tolerance);
        // double currentX = limelightSubsystem.getX();
        // double currentA = limelightSubsystem.getA();

        // double rot = aimPIDController.calculate(currentX, 0);
        // double range = rangePIDController.calculate(currentA, LimelightSubsystem.targetArea);
        // Translation2d translation = new Translation2d(range, range);

        // driveTrain.drive(translation, rot, false);

        ChassisSpeeds speeds = holonomicController.calculate(
            driveTrain.getPose(),
            limelight.getTargetPose(DriveConstants.targetOffset),
            DriveConstants.approachSpeed,
            limelight.getTargetPose(DriveConstants.targetOffset).getRotation());
        Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double rotation = speeds.omegaRadiansPerSecond;

        limelight.isTargetingStarted = true;
        limelight.targetPosition = limelight.getTargetPose(DriveConstants.targetOffset);
        limelight.lastPosition = driveTrain.getPose();
        limelight.atTargetPosition = false;

        driveTrain.drive(translation, rotation, false);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return limelight.atTarget();
    }
}