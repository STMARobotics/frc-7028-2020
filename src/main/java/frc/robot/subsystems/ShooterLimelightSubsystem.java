package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.LimeLightConstants;

/**
 * Extends LimelightSubsystem to add information about the Power Port target distance.
 */
public class ShooterLimelightSubsystem extends LimelightSubsystem {
  public final LimelightConfig limelightConfig;

  public ShooterLimelightSubsystem(LimelightConfig limelightConfig) {
    super(limelightConfig.getNetworkTableName());
    this.limelightConfig = limelightConfig;
  }

  @Override
  public ShuffleboardLayout addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = super.addDashboardWidgets(dashboard);
    detailDashboard.addNumber("Distance", () -> Units.metersToInches(getDistanceToTarget()));
    return detailDashboard;
  }

  private double getLimelightDistanceToTarget() {
    if (getTargetAcquired()) {
      return (LimeLightConstants.TARGET_HEIGHT - limelightConfig.getMountHeight())
          / Math.tan(Units.degreesToRadians(limelightConfig.getMountAngle() + getTargetY()));
    }
    return 0.0;
  }

  public double getDistanceToTarget() {
    return Math.sqrt(Math.pow(getLimelightDistanceToTarget(), 2)
        + Math.pow(limelightConfig.getMountDistanceFromCenter(), 2)) - limelightConfig.getMountDepth();
  }

}