package frc.robot.subsystems;

import static frc.robot.Constants.LimeLightConstants.TARGET_ACQUIRED;
import static frc.robot.Constants.LimeLightConstants.TARGET_X_MAX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {

  private double targetX = 0.0;
  private boolean targetAcquired = false;

  public LimelightSubsystem() {
    NetworkTableInstance.getDefault().getTable("limelight").addEntryListener("tl",
        (table, key, entry, value, flags) -> {
          updateValues(table);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  private void updateValues(NetworkTable table) {
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

}