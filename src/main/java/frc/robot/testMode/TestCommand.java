package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TestModeConstants;

public class TestCommand extends CommandBase
{

  public NetworkTableEntry getTestModeEntry(String name) {
    return NetworkTableInstance.getDefault().getTable(TestModeConstants.NetworkTableName).getEntry(name);
  }
}