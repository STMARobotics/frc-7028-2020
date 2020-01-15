package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * DriveToTargetCommand
 */
public class DriveToTargetCommand extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final Pose2d targetPosition;
  private Trajectory trajectory;
  private double targetX;
  private double targetY;
  private double targetOrientation;
  private double currentX;
  private double currentY;

  public DriveToTargetCommand(DriveTrainSubsystem driveTrainSubsystem, Pose2d targetPosition) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   driveTrainSubsystem.getCurrentPose(),
    //   null,
    //   targetPosition,
    //   null);
    System.out.println("Started Command");
  }
  
  @Override
  public void execute() {
    currentX = driveTrainSubsystem.getCurrentPose().getTranslation().getX();
    currentY = driveTrainSubsystem.getCurrentPose().getTranslation().getY();
    double netX = currentX - targetX;
    double netY = currentY - targetY;
    double pointOrientation;
    if (netX >= 0) {
      if (netY >= 0) {
        pointOrientation = -90 - Math.atan(netX/netY);
      } else {
        pointOrientation = 90 + Math.atan(netX/-netY);
      }
    } else {
      if (netY >= 0) {
        pointOrientation = -90 + Math.atan(-netX/netY);
      } else {
        pointOrientation = 90 - Math.atan(-netX/-netY);
      }
     }
     SmartDashboard.putNumber("Target Orientation: ", pointOrientation);
     if (!(driveTrainSubsystem.getCurrentPose().getRotation().getDegrees() <= pointOrientation + 3 && driveTrainSubsystem.getCurrentPose().getRotation().getDegrees() >= pointOrientation - 3)) {
       driveTrainSubsystem.arcadeDrive(0, .15);
     } else {
       driveTrainSubsystem.arcadeDrive(.3, 0);
     }
  }

  @Override
  public boolean isFinished() {
    return (currentX >= targetX - .25 && currentX <= targetX + .25) &&
      (currentY >= targetY - .25 && currentY <= targetY + .25) /*&&
      (targetOrientation == gyroSubsystem.getGyroYaw() ||
      targetOrientation == gyroSubsystem.getGyroYaw())*/;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0, 0);
  }
  
}