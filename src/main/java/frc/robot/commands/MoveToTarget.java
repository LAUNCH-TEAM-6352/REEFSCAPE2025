package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.PIDConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** A command that uses Limelight to drive to a target using swerve drive. */
public class MoveToTarget extends Command
{
    private final LimelightSubsystem limelightSubsystem;
    private final DriveTrain driveTrain;
    private final String toleranceKey;
    private final PIDController aimPIDController;
    private final PIDController rangePIDController;


    // TODO: maybe implement lists depending on blue/red alliance

    public MoveToTarget(LimelightSubsystem limelightSubsystem, DriveTrain driveTrain, String toleranceKey)
    {
        this.limelightSubsystem = limelightSubsystem;
        this.driveTrain = driveTrain;
        this.toleranceKey = toleranceKey;
        addRequirements(limelightSubsystem, driveTrain);
        aimPIDController = new PIDController(0.08, 0.035, 0.025);
        rangePIDController = new PIDController(0.15, 0.05, 0.4);
    }

    @Override
    public void initialize()
    {
         var tolerance = SmartDashboard.getNumber(toleranceKey, PIDConstants.tolerance);
         double currentX = limelightSubsystem.getX();
         double currentA = limelightSubsystem.getA();

         double rot = aimPIDController.calculate(currentX, 0);
         double range = rangePIDController.calculate(currentA, LimelightSubsystem.targetArea);
         Translation2d translation = new Translation2d(range, range);
         // could possibly use the version with (distance, Rotation2d rotation) instead of (translation, rot)

         driveTrain.drive(translation, rot, false);
         // have to make fieldRelative a constant later
    }

    @Override
    public void execute()
    {
        
    }

    @Override
    public void end(boolean interrupted)
    {
        driveTrain.drive(new Translation2d(0, 0), 0, false);
    }

    @Override
    public boolean isFinished()
    {
        return limelightSubsystem.atTarget();
    }
}