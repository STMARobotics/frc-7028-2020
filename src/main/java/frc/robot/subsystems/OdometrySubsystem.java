package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * OdometrySubsystem
 */
public class OdometrySubsystem extends SubsystemBase {

  private final GyroSubsystem gyroSubsystem;

  private final DifferentialDriveOdometry differentialDriveOdometry;

  private Pose2d savedPosition;

  public OdometrySubsystem(GyroSubsystem gyroSubsystem) {
    this.gyroSubsystem = gyroSubsystem;
    this.differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyroSubsystem.getGyroYaw()));
    this.savedPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(gyroSubsystem.getGyroYaw()));
  }

  public Pose2d getCurrentPosition() {
    return differentialDriveOdometry.getPoseMeters();
  }

  public void savePosition() {
    savedPosition = getCurrentPosition();
  }

  public Pose2d getSavedPosition() {
    return savedPosition;
  }
  
}