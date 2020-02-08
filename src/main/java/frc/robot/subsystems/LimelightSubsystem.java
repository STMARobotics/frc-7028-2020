package frc.robot.subsystems;

import static frc.robot.Constants.LimeLightConstants.LEFT_DISTANCE_FROM_FRONT;
import static frc.robot.Constants.LimeLightConstants.LEFT_MOUNT_ANGLE;
import static frc.robot.Constants.LimeLightConstants.PIPELINE_INDEX_FAR;
import static frc.robot.Constants.LimeLightConstants.PIPELINE_INDEX_NEAR;
import static frc.robot.Constants.LimeLightConstants.RIGHT_DISTANCE_FROM_FRONT;
import static frc.robot.Constants.LimeLightConstants.RIGHT_MOUNT_ANGLE;
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

  private final ShuffleboardLayout dashboard = Dashboard.subsystemsTab.getLayout("Limelight", BuiltInLayouts.kList)
      .withSize(2, 3).withPosition(4, 0);
  private final ShuffleboardLayout detailDashboard = dashboard.getLayout("Target", BuiltInLayouts.kGrid)
      .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));

  private final NetworkTable leftLimeLightNetworkTable;
  private final NetworkTable rightLimeLightNetworkTable;
  private double leftTargetX = 0.0;
  private double leftTargetY = 0.0;
  private boolean leftTargetAcquired = false;
  private double rightTargetX = 0.0;
  private double rightTargetY = 0.0;
  private boolean rightTargetAcquired = false;
  private boolean enabled;

  public LimelightSubsystem() {
    dashboard.add(this);
    detailDashboard.addBoolean("Acquired", this::getLeftTargetAcquired);
    detailDashboard.addNumber("Distance", () -> Units.metersToInches(getLeftDistance()));
    detailDashboard.addNumber("X", this::getLeftTargetX);
    detailDashboard.addNumber("Y", this::getLeftTargetY);
    leftLimeLightNetworkTable = NetworkTableInstance.getDefault().getTable("leftLimelight");
    rightLimeLightNetworkTable = NetworkTableInstance.getDefault().getTable("rightLimelight");
    leftLimeLightNetworkTable.addEntryListener("tl", this::update,
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    rightLimeLightNetworkTable.addEntryListener("tl", this::update,
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    new Trigger(() -> RobotState.isEnabled()).whenActive(this::enable)
        .whenInactive(new InstantWhenDisabledCommand(this::disable));
  }

  private void update(final NetworkTable table, final String key, final NetworkTableEntry entry,
      final NetworkTableValue value, final int flags) {
    leftTargetAcquired = table.getEntry("tv").getDouble(0.0) == TARGET_ACQUIRED;
    leftTargetX = table.getEntry("tx").getDouble(0.0);
    leftTargetY = table.getEntry("ty").getDouble(0.0);

    table.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
    table.getEntry("camMode").setDouble(enabled ? 0.0 : 1.0);
  }

  public boolean getLeftTargetAcquired() {
    return leftTargetAcquired;
  }

  public boolean getRightTargetAcquired() {
    return rightTargetAcquired;
  }

  public double getLeftTargetX() {
    return leftTargetX;
  }

  public double getRightTargetX() {
    return rightTargetX;
  }

  public double getLeftTargetY() {
    return leftTargetY;
  }

  public double getRightTargetY() {
    return rightTargetY;
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

  public double getLeftDistance() {
    if (leftTargetAcquired) {
      return ((TARGET_HEIGHT - LEFT_MOUNT_ANGLE)
          / Math.tan(Units.degreesToRadians(LEFT_MOUNT_ANGLE + getLeftTargetY()))) - (LEFT_DISTANCE_FROM_FRONT);
    }
    return 0.0;
  }

  public double getRightDistance() {
    if (rightTargetAcquired) {
      return ((TARGET_HEIGHT - RIGHT_MOUNT_ANGLE)
          / Math.tan(Units.degreesToRadians(RIGHT_MOUNT_ANGLE + getRightTargetY()))) - (RIGHT_DISTANCE_FROM_FRONT);
    }
    return 0.0;
  }

  public double getDistanceToTarget() {
    if (rightTargetAcquired && leftTargetAcquired) {
      return (getRightDistance() + getLeftDistance()) / 2;
    } else if (!rightTargetAcquired && leftTargetAcquired) {
      return getLeftDistance();
    } else if (rightTargetAcquired && !leftTargetAcquired) {
      return getRightDistance();
    } else {
      return 0.0;
    }
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