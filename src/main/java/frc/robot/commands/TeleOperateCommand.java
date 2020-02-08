package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * TeleOperateCommand
 */
public class TeleOperateCommand extends CommandBase {

  private final XboxController operatorConsole;

  private final IndexerSubsystem indexerSubsystem;

  public TeleOperateCommand(XboxController operatorConsole, IndexerSubsystem indexerSubsystem) {
    Dashboard.commandsTab.add(this);
    this.operatorConsole = operatorConsole;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    double input = operatorConsole.getTriggerAxis(Hand.kRight);
    indexerSubsystem.runManually(input * Math.abs(input));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }

}