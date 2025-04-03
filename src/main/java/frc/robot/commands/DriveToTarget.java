package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

/** A command that uses Limelight to drive to a target using swerve drive. */
public class DriveToTarget extends Command
{
    private final Limelight limelight;
    private final DriveTrain driveTrain;

    private final PIDController rangePIDController;
    private final PIDController aimPIDController;

    public DriveToTarget(Limelight limelight, DriveTrain driveTrain)
    {
        this.limelight = limelight;
        this.driveTrain = driveTrain;
        addRequirements(limelight, driveTrain);

        this.rangePIDController = new PIDController(0.7, 0, 0);

        this.aimPIDController = new PIDController(0.4, 0, 0.01);
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

        // limelight.isTargetingStarted = true;
        // limelight.targetPosition = limelight.getTargetPose(DriveConstants.targetOffset);
        // limelight.lastPosition = driveTrain.getPose();
        // limelight.atTargetPosition = false;

    }

    @Override
    public void execute()
    {
        double speedX = -rangePIDController.calculate(limelight.getRobotPose().getX(),
            limelight.getRobotPose().plus(limelight.getTargetPose().minus(new Pose2d())).getX());
        double speedY = rangePIDController.calculate(-limelight.getRobotPose().getY(),
            limelight.getRobotPose().plus(limelight.getTargetPose().minus(new Pose2d())).getY());
        double speedR = aimPIDController.calculate(limelight.getRobotPose().getRotation().getDegrees(),
            limelight.getRobotPose().plus(limelight.getTargetPose().minus(new Pose2d())).getRotation().getDegrees());

        ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedR);
        Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double rotation = speeds.omegaRadiansPerSecond;

        driveTrain.drive(translation, rotation, true);
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return rangePIDController.atSetpoint() && aimPIDController.atSetpoint();
    }
}