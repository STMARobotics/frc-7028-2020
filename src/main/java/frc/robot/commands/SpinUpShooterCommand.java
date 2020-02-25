package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Spins up the shooter anticipating to shoot a specific distance until the command is interrupted.
 */
public class SpinUpShooterCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final int distanceToTarget;

  public SpinUpShooterCommand(int distanceToTarget, ShooterSubsystem shooterSubsystem) {
    this.distanceToTarget = distanceToTarget;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.prepareToShoot(distanceToTarget);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

}