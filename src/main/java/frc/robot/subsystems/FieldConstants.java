package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldConstants
{
    public Pose2d closestBranch;

    public Pose2d[] branchPosesBlue = {
                    new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(-60)), // L
                    new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(-60)), // K
                    new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(-120)), // J
                    new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(-120)), // I
                    new Pose2d(5.27, 4.19, Rotation2d.fromDegrees(180)), // H
                    new Pose2d(5.27, 3.86, Rotation2d.fromDegrees(180)), // G
                    new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120)), // F
                    new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120)), // E
                    new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60)), // D
                    new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60)), // C
                    new Pose2d(3.71, 3.86, Rotation2d.fromDegrees(0)), // B
                    new Pose2d(3.71, 4.19, Rotation2d.fromDegrees(0)), // A
    };
    public Pose2d[] branchPosesRed = {
                    new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120)), // L
                    new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120)), // K
                    new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60)), // J
                    new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60)), // I
                    new Pose2d(12.29, 3.86, Rotation2d.fromDegrees(0)), // H
                    new Pose2d(12.29, 4.19, Rotation2d.fromDegrees(0)), // G
                    new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(-60)), // F
                    new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(-60)), // E
                    new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(-120)), // D
                    new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(-120)), // C
                    new Pose2d(13.84, 4.19, Rotation2d.fromDegrees(180)), // B
                    new Pose2d(13.84, 3.86, Rotation2d.fromDegrees(180)), // A
    };

    public Pose2d[] getReefPoses()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.get() == Alliance.Red ? branchPosesRed : branchPosesBlue;
    }

    public Pose2d getNearestBranch()
    {
        closestBranch = DriveTrain.getInstance().curPose.nearest(List.of(getReefPoses()));
        return closestBranch;
    }

    private static FieldConstants instance;

    public static FieldConstants getInstance()
    {
        if (instance == null)
            instance = new FieldConstants();
        return instance;
    }
}
