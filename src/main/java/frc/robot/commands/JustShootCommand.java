package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Shoots at a target a specified distance away, at least a given number of
 * times. This command does NOT aim it just shoots.
 */
public class JustShootCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final int ballsToShoot;
  private final int distance;
  private final boolean rapidFire;
  private int ballsShot = 0;
  private boolean wasFull = false;
  private Timer endTimer = new Timer();

  /**
   * 
   * @param ballsToShoot
   * @param distance distance in INCHES
   * @param shooterSubsystem
   * @param indexerSubsystem
   */
  public JustShootCommand(
        int ballsToShoot, int distance, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    this(ballsToShoot, distance, false, shooterSubsystem, indexerSubsystem);
  }

  public JustShootCommand(int ballsToShoot, int distance, boolean rapidFire, ShooterSubsystem shooterSubsystem,
      IndexerSubsystem indexerSubsystem) {
    
    this.ballsToShoot = ballsToShoot;
    this.distance = distance;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.rapidFire = rapidFire;

    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    ballsShot = 0;
    wasFull = indexerSubsystem.isFull();
    endTimer.reset();
  }

  @Override
  public void execute() {
    shooterSubsystem.prepareToShoot(distance);
    if (shooterSubsystem.isReadyToShoot() || (rapidFire && ballsShot > 0)) {
      indexerSubsystem.shoot();
    } else {
      indexerSubsystem.prepareToShoot();
    }
    var isFull = indexerSubsystem.isFull();
    if ((wasFull && !isFull) && (++ballsShot >= ballsToShoot)) {
      endTimer.start();
    }
    wasFull = isFull;
  }

  @Override
  public boolean isFinished() {
    return ballsShot >= ballsToShoot && endTimer.hasElapsed(ShooterConstants.SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    indexerSubsystem.stopIndexer();
  }

  public int getBallsShot() {
    return ballsShot;
  }
}