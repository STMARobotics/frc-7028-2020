package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * Calibrates the control panel arm and then lowers it
 */
public class CalibrateAndLowerArmCommand extends CommandBase {

  private final ControlPanelSubsystem controlPanelSubsystem;

  public CalibrateAndLowerArmCommand(ControlPanelSubsystem controlPanelSubsystem) {
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

  @Override
  public void end(boolean interrupted) {
    new RunCommand(controlPanelSubsystem::lowerArmPeriodic, controlPanelSubsystem).schedule();
  }

}