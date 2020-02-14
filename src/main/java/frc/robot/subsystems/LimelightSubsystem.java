package frc.robot.subsystems;

import static frc.robot.Constants.LimeLightConstants.PIPELINE_INDEX_FAR;
import static frc.robot.Constants.LimeLightConstants.PIPELINE_INDEX_NEAR;
import static frc.robot.Constants.LimeLightConstants.TARGET_ACQUIRED;
import static frc.robot.Constants.LimeLightConstants.TARGET_X_MAX;

import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.InstantWhenDisabledCommand;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable limelightNetworkTable;
  private final LimelightConfig limelightConfig;

  private double targetX = 0.0;
  private double targetY = 0.0;
  private boolean targetAcquired = false;
  private boolean enabled;

  public LimelightSubsystem(LimelightConfig limelightConfig) {
    this.limelightConfig = limelightConfig;
    
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(limelightConfig.getNetworkTableName());
    limelightNetworkTable.addEntryListener("tl", this::update, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    new Trigger(() -> RobotState.isEnabled()).whenActive(this::enable)
        .whenInactive(new InstantWhenDisabledCommand(this::disable));
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Target", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
    detailDashboard.addBoolean("Acquired", this::getTargetAcquired);
    detailDashboard.addNumber("Distance", () -> Units.metersToInches(getDistanceToTarget()));
    detailDashboard.addNumber("X", this::getTargetX);
    detailDashboard.addNumber("Y", this::getTargetY);
  }

  private void update(final NetworkTable table, final String key, final NetworkTableEntry entry,
      final NetworkTableValue value, final int flags) {
    targetAcquired = table.getEntry("tv").getDouble(0.0) == TARGET_ACQUIRED;
    targetX = table.getEntry("tx").getDouble(0.0);
    targetY = table.getEntry("ty").getDouble(0.0);

    table.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
    table.getEntry("camMode").setDouble(enabled ? 0.0 : 1.0);
  }

  public boolean getTargetAcquired() {
    return targetAcquired;
  }

  public double getTargetX() {
    // return targetX - getOffsetAngle();
    return targetX;
  }

  public double getTargetY() {
    return targetY;
  }

  public double getMaxX() {
    return TARGET_X_MAX;
  }

  /**
   * Turns the LEDS off and switches camera mode to driver.
   */
  public void disable() {
    enabled = false;
  }

  /**
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode
   * to vision processor.
   */
  public void enable() {
    enabled = true;
  }

  private double getLimelightDistanceToTarget() {
    if (targetAcquired) {
      return (LimeLightConstants.TARGET_HEIGHT - limelightConfig.getMountHeight())
          / Math.tan(Units.degreesToRadians(limelightConfig.getMountAngle() + getTargetY()));
    }
    return 0.0;
  }

  private double getOffsetAngle() {
    return 90.0 - 
        Math.toDegrees(Math.acos(getLimelightDistanceToTarget() / limelightConfig.getMountDistanceFromCenter()));
  }

  public double getDistanceToTarget() {
    return Math.sqrt(Math.pow(getLimelightDistanceToTarget(), 2)
        + Math.pow(limelightConfig.getMountDistanceFromCenter(), 2)) - limelightConfig.getMountDepth();
  }

  public void setProfile(final Profile profile) {
    switch (profile) {
      case NEAR:
        limelightNetworkTable.getEntry("pipeline").setDouble(PIPELINE_INDEX_NEAR);
        break;
      case MIDDLE:
      case FAR:
        limelightNetworkTable.getEntry("pipeline").setDouble(PIPELINE_INDEX_FAR);
    }
  }

}