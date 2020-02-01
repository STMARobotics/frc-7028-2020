package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHoodSubsystem;

/**
 * SetHoodCommand
 */
public class SetHoodCommand extends CommandBase{

  private final ShooterHoodSubsystem shooterHoodSubsystem;
  private final double hoodValue;

  public SetHoodCommand(ShooterHoodSubsystem shooterHoodSubsystem, double hoodValue) {
    this.shooterHoodSubsystem = shooterHoodSubsystem;
    this.hoodValue = hoodValue;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    shooterHoodSubsystem.setHoodValue(hoodValue);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}