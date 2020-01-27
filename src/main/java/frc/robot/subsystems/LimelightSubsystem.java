package frc.robot.subsystems;

import static frc.robot.Constants.LimeLightConstants.TARGET_ACQUIRED;
import static frc.robot.Constants.LimeLightConstants.TARGET_X_MAX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.InstantWhenDisabledCommand;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable limeLightNetworkTable;
  private double targetX = 0.0;
  private boolean targetAcquired = false;

  public LimelightSubsystem() {
    limeLightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    limeLightNetworkTable.addEntryListener("tl", this::update, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    
    new Trigger(() -> RobotState.isEnabled())
      .whenActive(this::enable)
      .whenInactive(new InstantWhenDisabledCommand(this::disable));
  }

  private void update(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
    targetAcquired = table.getEntry("tv").getDouble(0.0) == TARGET_ACQUIRED;
    targetX = table.getEntry("tx").getDouble(0.0);
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
    limeLightNetworkTable.getEntry("ledMode").setDouble(1.0);
    limeLightNetworkTable.getEntry("camMode").setDouble(1.0);
  }

  /**
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode to vision processor.
   */
  public void enable() {
    limeLightNetworkTable.getEntry("ledMode").setDouble(0.0);
    limeLightNetworkTable.getEntry("camMode").setDouble(0.0);
  }

}