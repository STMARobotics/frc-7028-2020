package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
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

  public DriveToTargetCommand(DriveTrainSubsystem driveTrainSubsystem,Pose2d targetPosition) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      driveTrainSubsystem.getCurrentPose(),
      null,
      targetPosition,
      null);
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
        pointOrientation = Math.toDegrees(Math.tan(deltaX / deltaY)) + 180;
      } else {
        pointOrientation = Math.toDegrees(Math.tan(deltaX / deltaY)) + 270;
      }
    } else {
      if (deltaY >= 0) {
        pointOrientation = Math.toDegrees(Math.tan(deltaX / deltaY)) + 90;
      } else {
        pointOrientation = Math.toDegrees(Math.tan(deltaX / deltaY));
      }
     }
     if (pointOrientation < -180 || pointOrientation > 180) {
       if (pointOrientation > 0) {
         pointOrientation -= 360;
       } else {
         pointOrientation += 360;
       }
     }
     if (driveTrainSubsystem.getYaw() >= pointOrientation + 5) {
       driveTrainSubsystem.arcadeDrive(0, .5);
     } else if (driveTrainSubsystem.getYaw() <= pointOrientation - 5) {
       driveTrainSubsystem.arcadeDrive(0, -.5);
     } else {
       driveTrainSubsystem.arcadeDrive(.5, 0);
     }
  }

  @Override
  public boolean isFinished() {
    return (currentX >= targetX - 5 && currentX <= targetX + 5) &&
      (currentY >= targetY - 5 && currentY <= targetY + 5) /*&&
      (targetOrientation == gyroSubsystem.getGyroYaw() ||
      targetOrientation == gyroSubsystem.getGyroYaw())*/;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0, 0);
  }
  
}