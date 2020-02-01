package frc.robot.commands;

import java.util.Collections;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * SetColorCommand
 */
public class SetColorCommand extends CommandBase {

  private final ShuffleboardLayout dashboard = 
      Dashboard.commandsTab.getLayout("Set Color", BuiltInLayouts.kList).withSize(2, 2).withPosition(6, 0);
  private final ShuffleboardLayout colorGrid = 
      dashboard.getLayout("Colors", BuiltInLayouts.kGrid).withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 1));
      //.withSize(2, 1).withPosition(0, 0);
  private final SuppliedValueWidget<Boolean> assignedColorWidget = 
      colorGrid.addBoolean("Assigned", () -> true).withProperties(Map.of("colorWhenTrue", "Black"));
  private final SuppliedValueWidget<Boolean> targetColorWidget =
      colorGrid.addBoolean("Target", () -> true).withProperties(Map.of("colorWhenTrue", "Black"));
  
  private static final Map<String, String> targetColorMap = Collections.unmodifiableMap(Map.of(
      "G", "Yellow",
      "B", "Red",
      "Y", "Green",
      "R", "Blue"
  ));

  private static final Map<String, String> colorNameMap = Collections.unmodifiableMap(Map.of(
      "G", "Green",
      "B", "Blue",
      "Y", "Yellow",
      "R", "Red"
  ));

  private final ControlPanelSubsystem controlPanelSubsystem;
  private String targetColor;

  public SetColorCommand(ControlPanelSubsystem controlPanelSubsystem) {
    dashboard.add(this);
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void initialize() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    targetColor = targetColorMap.get(gameData);
    targetColorWidget.withProperties(Map.of("colorWhenTrue", targetColor));
    assignedColorWidget.withProperties(Map.of("colorWhenTrue", colorNameMap.get(gameData)));
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