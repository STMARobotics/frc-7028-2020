package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * DriveToTargetCommand
 */
public class DriveToTargetCommand extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private double targetX;
  private double targetY;
  private double currentX;
  private double currentY;

  public DriveToTargetCommand(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    this.targetX = driveTrainSubsystem.getSavedPose().getTranslation().getX();
    this.targetY = driveTrainSubsystem.getSavedPose().getTranslation().getY();
  }
  
  @Override
  public void execute() {
    currentX = driveTrainSubsystem.getCurrentPose().getTranslation().getX();
    currentY = driveTrainSubsystem.getCurrentPose().getTranslation().getY();
    double netX = currentX - targetX;
    double netY = currentY - targetY;
    double pointOrientation;
    if (netX >= 0d) {
      if (netY >= 0d) {
        pointOrientation = -90d - Math.atan(netX/netY);
      } else {
        pointOrientation = 90d + Math.atan(netX/-netY);
      }
    } else {
      if (netY >= 0) {
        pointOrientation = -90d + Math.atan(-netX/netY);
      } else {
        pointOrientation = 90d - Math.atan(-netX/-netY);
      }
     }
     if (driveTrainSubsystem.getCurrentPose().getRotation().getDegrees() <= pointOrientation + 3 &&
        driveTrainSubsystem.getCurrentPose().getRotation().getDegrees() >= pointOrientation - 3) {
      driveTrainSubsystem.arcadeDrive(.3, 0.0);
     } else {
       driveTrainSubsystem.arcadeDrive(0.0, .15);
     }
  }

  @Override
  public boolean isFinished() {
    return (currentX >= targetX - .25 && currentX <= targetX + .25) &&
      (currentY >= targetY - .25 && currentY <= targetY + .25);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0.0, 0.0);
  }
  
}