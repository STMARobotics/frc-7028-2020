package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JetsonSubsystem extends SubsystemBase {

  private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("JetsonDetect");

  private JetsonDetection closestDetection;

  private ObjectMapper objectMapper = new ObjectMapper();

  public JetsonSubsystem() {
    addJetsonUpdateListener(networkTable, this::updateClosest, "Closest Detection");
  }

  private void addJetsonUpdateListener(NetworkTable limelightTable, TableEntryListener listener, String... keys) {
    for (String key : keys) {
      networkTable.addEntryListener(key, listener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }

  private void updateClosest(final NetworkTable table, final String key, final NetworkTableEntry entry,
      final NetworkTableValue value, final int flags) {
    try {
      var detectionJson = value.getString();
      if (detectionJson == null || detectionJson.isEmpty()) {
        closestDetection = null;
      } else {
        closestDetection = objectMapper.readValue(value.getString(), JetsonDetection.class);
      }
    } catch (Exception e) {
      closestDetection = null;
    }
  }

  public JetsonDetection getClosestDetection() {
    return closestDetection;
  }
  public void setCargoColor(String color){
    networkTable.getEntry("cargoColor").setString(color);
  }
}
