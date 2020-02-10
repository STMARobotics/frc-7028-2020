package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * RotateWheelCommand
 */
public class RotateWheelCommand extends CommandBase {

  private final ControlPanelSubsystem controlPanelSubsystem;

  private SuppliedValueWidget<Boolean> initialColorWidget;
      
  private String initialColor;
  private int colorCount;
  private boolean wasLastInitial;

  public RotateWheelCommand(ControlPanelSubsystem controlPanelSubsystem) {
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    dashboard.addBoolean("Initial Color", () -> true).withProperties(Map.of("colorWhenTrue", "Black"));
    dashboard.addNumber("Color Count", () -> colorCount);
  }

  @Override
  public void initialize() {
    initialColor = controlPanelSubsystem.getColor();
    if (initialColorWidget != null) {
      initialColorWidget.withProperties(Map.of("colorWhenTrue", initialColor));
    }
    colorCount = 0;
    wasLastInitial = true;
  }

  @Override
  public void execute() {
    controlPanelSubsystem.spinForRotations();
    var currentColor = controlPanelSubsystem.getColor();
    if ("White".equals(initialColor)) {
      if (!"White".equals(currentColor)) {
        initialColor = currentColor;
      }
    } else {
      boolean isCurrentInitial = currentColor.equals(initialColor);
      if (!wasLastInitial && isCurrentInitial) {
        colorCount++;
      }
      wasLastInitial = isCurrentInitial;
    }
  }

  @Override
  public boolean isFinished() {
    return (colorCount >= 6) && !controlPanelSubsystem.getColor().equals(initialColor);
  }

  @Override
  public void end(boolean interrupted) {
    controlPanelSubsystem.stopWheel();
  }

}