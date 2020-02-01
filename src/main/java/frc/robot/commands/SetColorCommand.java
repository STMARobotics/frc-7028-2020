package frc.robot.commands;

import java.util.Collections;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * SetColorCommand
 */
public class SetColorCommand extends CommandBase {

  private static final Map<String, String> colorMap = Collections.unmodifiableMap(Map.of(
      "G", "Yellow",
      "B", "Red",
      "Y", "Green",
      "R", "Blue"
  ));

  private final ControlPanelSubsystem controlPanelSubsystem;
  private String targetColor;

  public SetColorCommand(ControlPanelSubsystem controlPanelSubsystem) {
    Dashboard.commandsTab.add(this);
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void initialize() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    targetColor = colorMap.get(gameData);
  }

  @Override
  public void execute() {
    controlPanelSubsystem.spinForColor();
  }

  @Override
  public boolean isFinished() {
    if (targetColor == null) {
      return true;
    }
    return targetColor == controlPanelSubsystem.getColor();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      controlPanelSubsystem.stopWheel();
    } else {
      controlPanelSubsystem.stopHere();
    }
  }

}