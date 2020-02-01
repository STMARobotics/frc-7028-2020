package frc.robot.subsystems;

import static frc.robot.Constants.LimeLightConstants.TARGET_ACQUIRED;
import static frc.robot.Constants.LimeLightConstants.TARGET_X_MAX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Dashboard;
import frc.robot.commands.InstantWhenDisabledCommand;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {

  private final ShuffleboardLayout dashboard = Dashboard.subsystemsTab.getLayout("Limelight", BuiltInLayouts.kList)
      .withSize(2, 2).withPosition(4, 0);

  private final NetworkTable limeLightNetworkTable;
  private double targetX = 0.0;
  private boolean targetAcquired = false;
  private boolean enabled;

  public LimelightSubsystem() {
    dashboard.add(this);
    limeLightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    limeLightNetworkTable.addEntryListener("tl", this::update, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    
    new Trigger(() -> RobotState.isEnabled())
      .whenActive(this::enable)
      .whenInactive(new InstantWhenDisabledCommand(this::disable));
  }

  private void update(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
    targetAcquired = table.getEntry("tv").getDouble(0.0) == TARGET_ACQUIRED;
    targetX = table.getEntry("tx").getDouble(0.0);
    
    table.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
    table.getEntry("camMode").setDouble(enabled ? 0.0 : 1.0);
  }

  public boolean getTargetAcquired() {
    return targetAcquired;
  }

  public double getTargetX() {
    return targetX;
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
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode to vision processor.
   */
  public void enable() {
    enabled = true;
  }

}