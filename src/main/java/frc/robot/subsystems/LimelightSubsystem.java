package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public LimelightSubsystem() {}

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight Target ID", getTargetID());
        SmartDashboard.putNumber("Limelight Yaw", getYaw()); // horizontal offset
        SmartDashboard.putNumber("Limelight Pitch", getPitch()); // vertical offset
        SmartDashboard.putNumber(
            "Distance to Target in meters", 
            getDistanceToTargetMeters(
                LimelightConstants.CAMERA_HEIGHT_METERS, 
                LimelightConstants.TARGET_HEIGHT_METERS, 
                LimelightConstants.CAMERA_PITCH_RADIANS
            )
        );
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0; // if target is visible, value of 1
    }

    public double getTargetID() {
        return limelightTable.getEntry("tid").getDouble(-1); // no valid target ID, value of -1
    }

    public double getYaw() {
        return limelightTable.getEntry("tx").getDouble(0.0); // horizontal offset 
    }

    public double getPitch() {
        return limelightTable.getEntry("ty").getDouble(0.0); // vertical offset 
    }

    public double getDistanceToTargetMeters(double cameraHeight, double targetHeight, double cameraPitchRadians) {
        if (!hasTarget()) {
            return 0.0;
        }
        double targetAngleRadians = Math.toRadians(getPitch());
        /*  basically a rearrangement of tan(0) = opp/adj
        - 0 is the angle from the camera to the target, considering both internal angle deviations
        - opp is the difference in height between the camera and the target
        - adj is the distance from the camera to the target (what we're solving for)
        */
        return (targetHeight - cameraHeight) / Math.tan(cameraPitchRadians + targetAngleRadians);
    }
}