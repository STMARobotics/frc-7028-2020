package frc.robot.commands;

import static frc.robot.Constants.AimConstants.kD;
import static frc.robot.Constants.AimConstants.kP;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Aims at the target and shoots at least a given number of times.
 */
public class JustShootCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final PIDController pidController = new PIDController(kP, 0, kD);
  
  private final int ballsToShoot;
  private final int distance;
  private int ballsShot = 0;
  private boolean noTarget = false;
  private boolean wasFull = false;
  private Timer endTimer = new Timer();

  /**
   * 
   * @param ballsToShoot
   * @param distance distance in INCHES
   * @param shooterSubsystem
   * @param indexerSubsystem
   */
  public JustShootCommand(int ballsToShoot, int distance, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {

    this.ballsToShoot = ballsToShoot;
    this.distance = distance;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(shooterSubsystem, indexerSubsystem);

    pidController.setTolerance(AimConstants.AIM_TOLERANCE);
  }

  @Override
  public void initialize() {
    super.initialize();
    noTarget = false;
    ballsShot = 0;
    wasFull = indexerSubsystem.isFull();
    endTimer.reset();
    pidController.reset();
  }

  @Override
  public void execute() {
    super.execute();
    shooterSubsystem.prepareToShoot(distance);
    if (shooterSubsystem.isReadyToShoot() && pidController.atSetpoint()) {
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
    return noTarget || (ballsShot >= ballsToShoot && endTimer.hasElapsed(ShooterConstants.SHOOT_TIME));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooterSubsystem.stopShooter();
    indexerSubsystem.stopIndexer();
  }
}