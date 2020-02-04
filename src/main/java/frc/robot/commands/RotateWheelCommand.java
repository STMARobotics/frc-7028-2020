package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * RotateWheelCommand
 */
public class RotateWheelCommand extends CommandBase {

  private final ControlPanelSubsystem controlPanelSubsystem;

  private final ShuffleboardLayout dashboard = 
      Dashboard.commandsTab.getLayout("Rotate Wheel", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
  private final SuppliedValueWidget<Boolean> initialColorWidget = 
      dashboard.addBoolean("Initial Color", () -> true).withProperties(Map.of("colorWhenTrue", "Black"));
  
  private String initialColor;
  private int colorCount;
  private boolean wasLastInitial;

  public RotateWheelCommand(ControlPanelSubsystem controlPanelSubsystem) {
    dashboard.add(this);
    dashboard.addNumber("Color Count", () -> colorCount);
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void initialize() {
    initialColor = controlPanelSubsystem.getColor();
    initialColorWidget.withProperties(Map.of("colorWhenTrue", initialColor));
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