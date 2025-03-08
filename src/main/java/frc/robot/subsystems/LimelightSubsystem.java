package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase
{

    public static final double kMaxSpeed = 0.30;
    public static final double kMaxAngularSpeed = Math.PI * 0.2;
    public static final double targetArea = 25;

    public boolean isTargetingStarted;
    public Pose2d targetPosition;
    public Transform2d tolerance = DriveConstants.tolerance;
    public Pose2d lastPosition;
    public boolean atTargetPosition;

    public LimelightSubsystem()
    {
    }

    public boolean hasTarget()
    {
        return LimelightHelpers.getTV("limelight"); // if target is visible, value of 1
    }

    public double getTargetID()
    {
        return LimelightHelpers.getFiducialID("limelight"); // no valid target ID, value of -1
    }

    public double getX()
    {
        return LimelightHelpers.getTX("limelight"); // horizontal offset in degrees
    }

    public double getY()
    {
        return LimelightHelpers.getTY("limelight"); // vertical offset in degrees
    }

    public double getA()
    {
        return LimelightHelpers.getTA("limelight");
    }

    public boolean atTarget()
    {
        return getA() >= targetArea;
    }

    public double getDistanceToTarget(double cameraHeight, double targetHeight)
    {
        if (!hasTarget())
        {
            return 0.0;
        }
        return (targetHeight - cameraHeight) / Math.tan(getY());
    }

    public Pose2d getTargetPose(double offsetDistance)
    {
        double[] tagPoseArray = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        if (tagPoseArray == null)
            return null;
        Pose3d tagPose = new Pose3d(tagPoseArray[0], tagPoseArray[1], tagPoseArray[2],
            new Rotation3d(tagPoseArray[3], tagPoseArray[4], tagPoseArray[5]));
        Translation3d offset = new Translation3d(offsetDistance, 0.0, 0.0);
        Pose3d targetPose = tagPose.transformBy(new Transform3d(offset, new Rotation3d()));
        return targetPose.toPose2d();
    }

    public Pose2d getRobotPose()
    {
        double[] botPose = LimelightHelpers.getBotPose_wpiBlue("limelight");
        if (botPose == null)
            return null;
        Pose3d tagPose = new Pose3d(botPose[0], botPose[1], botPose[2],
            new Rotation3d(botPose[3], botPose[4], botPose[5]));
        return tagPose.toPose2d();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run

        var position = getRobotPose();

        if (isTargetingStarted)
        {
            if (((position.minus(targetPosition).getX()) < tolerance.getX()
                && (position.minus(targetPosition).getY()) < tolerance.getY()
                && (position.minus(targetPosition).getRotation().getRadians()) < tolerance.getRotation().getRadians())

                && ((position.minus(lastPosition).getX()) < tolerance.getX()
                    && (position.minus(lastPosition).getY()) < tolerance.getY()
                    && (position.minus(lastPosition).getRotation().getRadians()) < tolerance.getRotation()
                        .getRadians()))
            {
                atTargetPosition = true;
                isTargetingStarted = false;
            }
            else
            {
                lastPosition = position;
            }
        }
    }
}