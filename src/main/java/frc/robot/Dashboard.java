package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Dashboard
 */
public class Dashboard {

  public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  public static final ShuffleboardTab subsystemsTab = Shuffleboard.getTab("Subsystems");
  public static final ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands");
  public static final ShuffleboardTab limelightsTab = Shuffleboard.getTab("Limelights");

}