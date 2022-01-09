package frc.robot.commands;

import java.util.Collections;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * SetColorCommand
 */
public class SetColorCommand extends CommandBase {

  private SuppliedValueWidget<Boolean> assignedColorWidget;
  private SuppliedValueWidget<Boolean> targetColorWidget;
  
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
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var colorGrid = dashboard.getLayout("Colors", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 1));
    assignedColorWidget = colorGrid.addBoolean("Assigned", () -> true).withProperties(Map.of("colorWhenTrue", "Black"));
    targetColorWidget = colorGrid.addBoolean("Target", () -> true).withProperties(Map.of("colorWhenTrue", "Black"));
  }

  @Override
  public void initialize() {
    var gameData = DriverStation.getGameSpecificMessage();
    targetColor = targetColorMap.get(gameData);
    if (targetColorWidget != null && assignedColorWidget != null) {
      targetColorWidget.withProperties(Map.of("colorWhenTrue", targetColor));
      assignedColorWidget.withProperties(Map.of("colorWhenTrue", colorNameMap.get(gameData)));
    }
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