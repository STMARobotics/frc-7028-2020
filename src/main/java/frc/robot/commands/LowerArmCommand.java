package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * Lowers the control panel arm fully
 */
public class LowerArmCommand extends CommandBase {

  private final ControlPanelSubsystem controlPanelSubsystem;

  public LowerArmCommand(ControlPanelSubsystem controlPanelSubsystem) {
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void execute() {
    controlPanelSubsystem.lowerArm();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Lowered arm");
  }

  @Override
  public boolean isFinished() {
    return controlPanelSubsystem.isArmDown();
  }

}