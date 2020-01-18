package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * RotateWheelCommand
 */
public class RotateWheelCommand extends CommandBase {

  private final ControlPanelSubsystem controlPanelSubsystem;

  private String initialColor;
  private int colorCount;
  private boolean wasLastInitial;

  public RotateWheelCommand(ControlPanelSubsystem controlPanelSubsystem) {
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void initialize() {
    initialColor = controlPanelSubsystem.getColor();
    colorCount = 0;
    wasLastInitial = true;
  }

  @Override
  public void execute() {
    controlPanelSubsystem.spinWheel();
    boolean isCurrentInitial = controlPanelSubsystem.getColor().equals(initialColor);
    controlPanelSubsystem.getColor();
    if (!wasLastInitial && isCurrentInitial) {
      colorCount++;
    }
    wasLastInitial = isCurrentInitial;
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