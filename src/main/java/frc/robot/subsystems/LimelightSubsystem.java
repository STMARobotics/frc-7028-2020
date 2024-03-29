package frc.robot.subsystems;

import static frc.robot.Constants.LimeLightConstants.TARGET_ACQUIRED;
import static frc.robot.Constants.LimeLightConstants.TARGET_X_MAX;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.InstantWhenDisabledCommand;
import frc.robot.networktables.DoubleEntryValue;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase implements ILimelightSubsystem {
  //valid keys - https://docs.limelightvision.io/en/latest/networktables_api.html
  private final static String ntPipelineLatency = "tl";
  private final static String ntTargetValid = "tv";
  private final static String ntTargetX = "tx";
  private final static String ntTargetY = "ty";

  private final NetworkTable limelightNetworkTable;
  private final String networkTableName;

  private long lastLatencyUpdate = System.currentTimeMillis();

  private DoubleEntryValue targetValid = new DoubleEntryValue(0);
  private long targetLastSeen = 0;
  private DoubleEntryValue targetX = new DoubleEntryValue(0);
  private DoubleEntryValue targetY = new DoubleEntryValue(0);

  private final HashMap<String, MedianFilter> updateFilterMap = new HashMap<>();
  
  private boolean enabled;
  private Profile activeProfile = Profile.NEAR;

  public LimelightSubsystem(String networkTableName) {
    
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
    this.networkTableName = networkTableName;

    //this adds listeners on an explicit list
    addLimelightUpdateListeners(limelightNetworkTable, ntPipelineLatency, ntTargetValid, ntTargetX, ntTargetY);

    new Trigger(RobotState::isEnabled).whenInactive(new InstantWhenDisabledCommand(this::disable));
  }

  private void addLimelightUpdateListeners(NetworkTable limelightTable, String... keys) {
    for (String key : keys) {
      limelightNetworkTable.addEntryListener(key, this::update, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
      updateFilterMap.putIfAbsent(key, new MedianFilter(20));
    }
  }

  public ShuffleboardLayout addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Target", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
    detailDashboard.addBoolean("Acquired", this::getTargetAcquired);
    detailDashboard.addNumber("X", this::getTargetX);
    detailDashboard.addNumber("Y", this::getTargetY);
    return detailDashboard;
  }

  private void update(final NetworkTable table, final String key, final NetworkTableEntry entry,
      final NetworkTableValue value, final int flags) {

    boolean shouldFlush = false;
    long updateMs = 0;
    switch(key) {

      case ntTargetX:
        var previousX = targetX;
        targetX = new DoubleEntryValue(value.getDouble());

        updateMs = targetX.updateTime - previousX.updateTime;
      break;
      case ntTargetY:
        var previousY = targetY;
        targetY = new DoubleEntryValue(value.getDouble());

        updateMs = targetY.updateTime - previousY.updateTime;
      break;

      case ntPipelineLatency:
        var previousLatencyUpdate = lastLatencyUpdate;
        lastLatencyUpdate = System.currentTimeMillis();
        updateMs = lastLatencyUpdate - previousLatencyUpdate; //update for debugging data

        //we could maybe move these to a command that watches the subsystem, might be more consisent depending how often this is getting hit
        // Flush NetworkTable to send LED mode and pipeline updates immediately
        shouldFlush = (table.getEntry("ledMode").getDouble(0.0) != (enabled ? 0.0 : 1.0) || 
          limelightNetworkTable.getEntry("pipeline").getDouble(0.0) != activeProfile.pipelineId);

        table.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
        table.getEntry("camMode").setDouble(enabled ? 0.0 : 1.0);
        limelightNetworkTable.getEntry("pipeline").setDouble(activeProfile.pipelineId);
      break;

      case ntTargetValid:
        var previousTargetValid = targetValid;
        targetValid = new DoubleEntryValue(value.getDouble());

        if (targetValid.value == TARGET_ACQUIRED) {
          targetLastSeen = targetValid.updateTime;
        }

        updateMs = targetValid.updateTime - previousTargetValid.updateTime;
      break;
    }

    //write out debug information so we can get an idea how often these updates are coming through from the limelight
    if (updateMs > 0 && updateFilterMap.containsKey(key)) {

      NetworkTableInstance.getDefault().getTable("LimelightDebug").getSubTable(networkTableName)
        .getEntry(key + "_updateFrequencyMs").setNumber(updateFilterMap.get(key).calculate(updateMs));
    }

    if (shouldFlush)  {
      NetworkTableInstance.getDefault().flush();
    }
  }

  public boolean getTargetAcquired() {
    return targetValid.value == TARGET_ACQUIRED;
  }

  public long getTargetLastSeen() {
    return targetLastSeen;
  }

  public double getTargetX() {
    // return targetX - getOffsetAngle();
    return targetX.value;
  }

  public double getTargetY() {
    return targetY.value;
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

  public void setProfile(final Profile profile) {
    activeProfile = profile;
  }

  public Profile getProfile() {
    return activeProfile;
  }

  public String getNetworkTableName() {
    return networkTableName;
  }

}