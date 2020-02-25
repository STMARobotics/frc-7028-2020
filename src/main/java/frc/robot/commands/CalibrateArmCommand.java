package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * Calibrates the control panel arm and then lowers it
 */
public class CalibrateArmCommand extends CommandBase {

  private final ControlPanelSubsystem controlPanelSubsystem;

  public CalibrateArmCommand(ControlPanelSubsystem controlPanelSubsystem) {
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void execute() {
    controlPanelSubsystem.calibrateArm();
  }

  @Override
  public boolean isFinished() {
    return controlPanelSubsystem.isArmUp();
  }

}