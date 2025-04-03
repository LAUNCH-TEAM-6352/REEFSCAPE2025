package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase
{

    public static final double kMaxSpeed = 0.30;
    public static final double kMaxAngularSpeed = Math.PI * 0.2;
    public static final double targetArea = 25;

    public boolean isTargetingStarted;
    public Pose2d targetPosition;
    public Transform2d tolerance = DriveConstants.tolerance;
    public Pose2d lastPosition;
    public boolean atTargetPosition;

    Pose2d botPose;

    public Limelight()
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

    public Pose2d getTargetPose()
    {
        // double[] tagPoseArray = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        // if (tagPoseArray == null)
        // return null;
        // Transform2d tagPose = new Transform2d(tagPoseArray[0], tagPoseArray[1],
        // new Rotation2d(Math.toRadians(tagPoseArray[5])));
        // Translation2d offset = new Translation2d(offsetDistance, 0.0);
        // Transform2d targetPose = tagPose.plus(new Transform2d(offset, new Rotation2d()));
        // return targetPose;

        Pose2d fieldToBranch = FieldConstants.getInstance().getNearestBranch();
        Pose2d branchToRobot = new Pose2d(-0.6, 0, Rotation2d.kZero);
        return fieldToBranch.plus(branchToRobot.minus(new Pose2d()));
    }

    public Pose2d getRobotPose()
    {
        // double[] botPoseArray = LimelightHelpers.getBotPose_wpiBlue("limelight");
        // if (botPoseArray == null)
        // return null;
        // Pose2d botPose = new Pose2d(botPoseArray[0], botPoseArray[1],
        // new Rotation2d(Math.toRadians(botPoseArray[5])));
        // return botPose;

        botPose = DriveTrain.getInstance().curPose;

        return botPose;
    }

    public void update()
    {
        LimelightHelpers.SetRobotOrientation("limelight",
            DriveTrain.getInstance().curPose.getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        DriveTrain.getInstance().swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(botPose.pose,
            Utils.fpgaToCurrentTime(botPose.timestampSeconds));
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run

        // var position = getRobotPose();

        // var targetDifference = position.minus(targetPosition);
        // var lastDifference = position.minus(lastPosition);

        // if (isTargetingStarted)
        // {
        // if ((Math.abs(targetDifference.getX()) < tolerance.getX()
        // && Math.abs(targetDifference.getY()) < tolerance.getY()
        // && Math.abs(targetDifference.getRotation().getRadians()) < tolerance.getRotation().getRadians())

        // && (Math.abs(lastDifference.getX()) < tolerance.getX()
        // && Math.abs(lastDifference.getY()) < tolerance.getY()
        // && Math.abs(lastDifference.getRotation().getRadians()) < tolerance.getRotation()
        // .getRadians()))
        // {
        // atTargetPosition = true;
        // isTargetingStarted = false;
        // }
        // else
        // {
        // lastPosition = position;
        // }
        // }
    }

    private static Limelight instance;

    public static Limelight getInstance()
    {
        if (instance == null)
            instance = new Limelight();
        return instance;
    }
}