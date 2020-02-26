package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * Raises the control panel arm fully
 */
public class RaiseArmCommand extends CommandBase {

  private final ControlPanelSubsystem controlPanelSubsystem;

  public RaiseArmCommand(ControlPanelSubsystem controlPanelSubsystem) {
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void execute() {
    controlPanelSubsystem.raiseArm();
  }

  @Override
  public boolean isFinished() {
    return controlPanelSubsystem.isArmUp();
  }

}