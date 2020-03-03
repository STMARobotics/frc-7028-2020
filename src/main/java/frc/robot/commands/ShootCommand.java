package frc.robot.commands;

import static frc.robot.Constants.AimConstants.kD;
import static frc.robot.Constants.AimConstants.kP;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterLimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Aims at the target and shoots at least a given number of times.
 */
public class ShootCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private final PIDController pidController = new PIDController(kP, 0, kD);
  
  private final int ballsToShoot;
  private final ShooterLimelightSubsystem limelightSubsystem;
  private int ballsShot = 0;
  private boolean wasFull = false;
  private Timer endTimer = new Timer();

  private MedianFilter yFilter = new MedianFilter(5);
  private MedianFilter xFilter = new MedianFilter(5);

  public ShootCommand(int ballsToShoot, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
      ShooterLimelightSubsystem limelightSubsystem, DriveTrainSubsystem driveTrainSubsystem) {

    this.ballsToShoot = ballsToShoot;
    this.limelightSubsystem = limelightSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(shooterSubsystem, indexerSubsystem, limelightSubsystem, driveTrainSubsystem);

    pidController.setTolerance(AimConstants.AIM_TOLERANCE);
  }

  @Override
  public void initialize() {
    super.initialize();
    ballsShot = 0;
    wasFull = indexerSubsystem.isFull();
    endTimer.reset();
    pidController.reset();
  }

  @Override
  public void execute() {
    super.execute();
    if (limelightSubsystem.getTargetAcquired()) {
      var filteredDistance = yFilter.calculate(limelightSubsystem.getDistanceToTarget());
      shooterSubsystem.prepareToShoot(Units.metersToInches(filteredDistance));
      aimShooter();
      if (shooterSubsystem.isReadyToShoot() && pidController.atSetpoint()) {
        indexerSubsystem.shoot();
      } else {
        indexerSubsystem.prepareToShoot();
      }
    } else {
      driveTrainSubsystem.stop();
    }
    var isFull = indexerSubsystem.isFull();
    if ((wasFull && !isFull) && (++ballsShot >= ballsToShoot)) {
      System.out.println("Shot all balls, waiting for time");
      endTimer.start();
    }
    wasFull = isFull;
  }

  private void aimShooter() {
    double filteredTargetX = xFilter.calculate(limelightSubsystem.getTargetX());
    double rotationSpeed = -pidController.calculate(filteredTargetX / 5);
    driveTrainSubsystem.arcadeDrive(0.0, rotationSpeed, false);
  }

  @Override
  public boolean isFinished() {
    return (ballsShot >= ballsToShoot && endTimer.hasElapsed(ShooterConstants.SHOOT_TIME));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooterSubsystem.stopShooter();
    indexerSubsystem.stopIndexer();
    driveTrainSubsystem.stop();
  }

  public int getBallsShot() {
    return ballsShot;
  }
}