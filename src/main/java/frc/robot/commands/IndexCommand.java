package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * IndexCommand
 */
public class IndexCommand extends CommandBase {

  private final IndexerSubsystem indexerSubsystem;

  public IndexCommand(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
  }

  @Override
  public void execute() {
    indexerSubsystem.intake();
  }
  
}