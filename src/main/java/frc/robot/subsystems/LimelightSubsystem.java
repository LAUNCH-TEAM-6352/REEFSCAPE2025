package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase
{

    public static final double kMaxSpeed = 0.30;
    public static final double kMaxAngularSpeed = Math.PI * 0.2;
    public static final double targetArea = 25;

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

    /*
     * public double limelight_aim_proportional() {
     * double kP = .01;
     * double tx = getX(); // Assume getTx() retrieves the tx value from the Limelight
     * return kP * tx * kMaxAngularSpeed * -1.0;
     * }
     * 
     * public double limelight_range_proportional() {
     * double kP = 1.0;
     * double tA = getA();
     * double speedDifferential = targetArea - tA;
     * double speedFactor = speedDifferential / targetArea;
     * double speed = kP * speedFactor * kMaxSpeed * -1.0;
     * System.out.println(speed);
     * return speed;
     * }
     */

    public boolean atTarget() {
        return getA() >= targetArea;
    }

    public double getDistanceToTarget(double cameraHeight, double targetHeight)
    {
        if (!hasTarget())
        {
            return 0.0;
        }
        /*
         * basically a rearrangement of tan(0) = opp/adj
         * - 0 is the angle from the camera to the target, considering both internal angle deviations
         * - opp is the difference in height between the camera and the target
         * - adj is the distance from the camera to the target (what we're solving for)
         */
        return (targetHeight - cameraHeight) / Math.tan(getY());
    }

    @Override
    public void periodic()
    {
        
    }
}