package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHoodSubsystem;

/**
 * SetHoodCommand
 */
public class SetHoodCommand extends CommandBase{

  private final ShooterHoodSubsystem shooterHoodSubsystem;
  private final double percentageExtended;

  public SetHoodCommand(ShooterHoodSubsystem shooterHoodSubsystem, double percentageExtended) {
    this.shooterHoodSubsystem = shooterHoodSubsystem;
    this.percentageExtended = percentageExtended;

    addRequirements(shooterHoodSubsystem);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    shooterHoodSubsystem.setPercentage(percentageExtended);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}