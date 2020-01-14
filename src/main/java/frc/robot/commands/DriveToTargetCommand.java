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
    double deltaX = currentX - targetX;
    double deltaY = currentY - targetY;
    double pointOrientation;
    if (deltaX >= 0) {
      if (deltaY >= 0) {
        pointOrientation = Math.toDegrees(Math.atan(deltaX / deltaY)) - 45;
      } else {
        pointOrientation = Math.toDegrees(Math.atan(deltaX / deltaY)) + 270;
      }
    } else {
      if (deltaY >= 0) {
        pointOrientation = Math.toDegrees(Math.atan(deltaX / deltaY)) + 90;
      } else {
        pointOrientation = Math.toDegrees(Math.atan(deltaX / deltaY));
      }
     }
     SmartDashboard.putNumber("Target Orientation: ", pointOrientation);
     if (pointOrientation < -180 || pointOrientation > 180) {
       if (pointOrientation > 0) {
         pointOrientation -= 360;
       } else {
         pointOrientation += 360;
       }
     }
     if (!(driveTrainSubsystem.getCurrentPose().getRotation().getDegrees() <= pointOrientation + 3 && driveTrainSubsystem.getCurrentPose().getRotation().getDegrees() >= pointOrientation - 3)) {
       driveTrainSubsystem.arcadeDrive(0, .1);
     } else {
       driveTrainSubsystem.arcadeDrive(-.3, 0);
     }
  }

  @Override
  public boolean isFinished() {
    return (currentX >= targetX - .1 && currentX <= targetX + .1) &&
      (currentY >= targetY - .1 && currentY <= targetY + .1) /*&&
      (targetOrientation == gyroSubsystem.getGyroYaw() ||
      targetOrientation == gyroSubsystem.getGyroYaw())*/;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0, 0);
  }
  
}