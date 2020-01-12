package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

/**
 * DriveToTargetCommand
 */
public class DriveToTargetCommand extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final OdometrySubsystem odometrySubsystem;
  private final GyroSubsystem gyroSubsystem;
  private final Pose2d targetPosition;
  private Trajectory trajectory;
  private double targetX;
  private double targetY;
  private double targetOrientation;
  private double currentX;
  private double currentY;

  public DriveToTargetCommand(DriveTrainSubsystem driveTrainSubsystem, OdometrySubsystem odometrySubsystem, 
    GyroSubsystem gyroSubsystem, Pose2d targetPosition) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.odometrySubsystem = odometrySubsystem;
    this.gyroSubsystem = gyroSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(driveTrainSubsystem);
    addRequirements(odometrySubsystem);
    addRequirements(gyroSubsystem);
  }

  @Override
  public void initialize() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      odometrySubsystem.getCurrentPosition(),
      null,
      targetPosition,
      null);
  }
  
  @Override
  public void execute() {
    currentX = odometrySubsystem.getCurrentPosition().getTranslation().getX();
    currentY = odometrySubsystem.getCurrentPosition().getTranslation().getY();
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
     if (gyroSubsystem.getGyroYaw() >= pointOrientation + 5) {
       driveTrainSubsystem.arcadeDrive(0, .5);
     } else if (gyroSubsystem.getGyroYaw() <= pointOrientation - 5) {
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