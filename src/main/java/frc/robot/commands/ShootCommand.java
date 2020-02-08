package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * ShootCommand
 */
public class ShootCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final LimelightSubsystem highLimelightSubsystem;
  private final LimelightSubsystem lowLimelightSubsystem;

  public ShootCommand(
      ShooterSubsystem shooterSubsystem, 
      IndexerSubsystem indexerSubsystem,
      LimelightSubsystem highLimelightSubsystem,
      LimelightSubsystem lowLimelightSubsystem) {
    
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.highLimelightSubsystem = highLimelightSubsystem;
    this.lowLimelightSubsystem = lowLimelightSubsystem;

    addRequirements(shooterSubsystem);
    addRequirements(indexerSubsystem);
    addRequirements(highLimelightSubsystem);
    addRequirements(lowLimelightSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.prepareToShoot(highLimelightSubsystem.getDistanceToTarget());
    if (shooterSubsystem.isReadyToShoot()) {
      indexerSubsystem.shoot();
    } else {
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    indexerSubsystem.stopIndexer();
  }

}