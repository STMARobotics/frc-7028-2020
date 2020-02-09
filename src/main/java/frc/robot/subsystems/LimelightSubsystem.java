package frc.robot.subsystems;

import static frc.robot.Constants.LimeLightConstants.PIPELINE_INDEX_FAR;
import static frc.robot.Constants.LimeLightConstants.PIPELINE_INDEX_NEAR;
import static frc.robot.Constants.LimeLightConstants.TARGET_ACQUIRED;
import static frc.robot.Constants.LimeLightConstants.TARGET_HEIGHT;
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
import frc.robot.Dashboard;
import frc.robot.commands.InstantWhenDisabledCommand;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {
  
  private final NetworkTable leftLimeLightNetworkTable;
  private final LimelightConfig limelightConfig;
  
  private final ShuffleboardLayout dashboard;
  private final ShuffleboardLayout detailDashboard;
  private double targetX = 0.0;
  private double targetY = 0.0;
  private boolean targetAcquired = false;
  private boolean enabled;

  public LimelightSubsystem(LimelightConfig limelightConfig) {
    dashboard = Dashboard.subsystemsTab.getLayout(limelightConfig.getNetworkTableName(), BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(4, 0);
    detailDashboard = dashboard.getLayout("Target", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
    dashboard.add(limelightConfig.getNetworkTableName(), this);
    this.limelightConfig = limelightConfig;
    detailDashboard.addBoolean("Acquired", this::getTargetAcquired);
    detailDashboard.addNumber("Distance", () -> Units.metersToInches(getDistanceToTarget()));
    detailDashboard.addNumber("X", this::getTargetX);
    detailDashboard.addNumber("Y", this::getTargetY);
    leftLimeLightNetworkTable = NetworkTableInstance.getDefault().getTable(limelightConfig.getNetworkTableName());
    leftLimeLightNetworkTable.addEntryListener("tl", this::update,
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    new Trigger(() -> RobotState.isEnabled()).whenActive(this::enable)
        .whenInactive(new InstantWhenDisabledCommand(this::disable));
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

  public double getDistanceToTarget() {
    if (targetAcquired) {
      return ((TARGET_HEIGHT - limelightConfig.getMountAngle())
          / Math.tan(Units.degreesToRadians(limelightConfig.getMountAngle() + getTargetY())))
          - (limelightConfig.getMountDepth());
    }
    return 0.0;
  }

  public void setProfile(final Profile profile) {
    switch (profile) {
      case NEAR:
        leftLimeLightNetworkTable.getEntry("pipeline").setDouble(PIPELINE_INDEX_NEAR);
        break;
      case MIDDLE:
      case FAR:
        leftLimeLightNetworkTable.getEntry("pipeline").setDouble(PIPELINE_INDEX_FAR);
    }
  }

}