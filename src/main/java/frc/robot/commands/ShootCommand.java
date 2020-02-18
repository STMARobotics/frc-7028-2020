package frc.robot.commands;

import static frc.robot.Constants.AimConstants.kD;
import static frc.robot.Constants.AimConstants.kF;
import static frc.robot.Constants.AimConstants.kP;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Aims at the target and shoots once.
 */
public class ShootCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final LimelightSubsystem highLimelightSubsystem;
  private final LimelightSubsystem lowLimelightSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private final PIDController pidController = new PIDController(kP, 0, kD);

  private boolean noTarget = false;
  private boolean shot = false;

  public ShootCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
      LimelightSubsystem highLimelightSubsystem, LimelightSubsystem lowLimelightSubsystem,
      DriveTrainSubsystem driveTrainSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.highLimelightSubsystem = highLimelightSubsystem;
    this.lowLimelightSubsystem = lowLimelightSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(shooterSubsystem, indexerSubsystem, highLimelightSubsystem, lowLimelightSubsystem,
        driveTrainSubsystem);

    pidController.setTolerance(.1);
  }

  @Override
  public void initialize() {
    noTarget = false;
    shot = false;
    pidController.reset();
    highLimelightSubsystem.enable();
    lowLimelightSubsystem.enable();
  }

  @Override
  public void execute() {
    if (highLimelightSubsystem.getTargetAcquired()) {
      shooterSubsystem.prepareToShoot(Units.metersToInches(highLimelightSubsystem.getDistanceToTarget()));
      aimShooter(highLimelightSubsystem);
      if (shooterSubsystem.isReadyToShoot() && pidController.atSetpoint()) {
        indexerSubsystem.shoot();
        shot = true;
      } else {
        indexerSubsystem.stopIndexer();
      }
    } else {
      noTarget = true;
      driveTrainSubsystem.arcadeDrive(0.0, 0.0, false);
    }
  }

  private void aimShooter(LimelightSubsystem selectedLimelightSubsystem) {
    double targetX = selectedLimelightSubsystem.getTargetX();
    double rotationSpeed = -pidController.calculate(targetX / selectedLimelightSubsystem.getMaxX());
    if (rotationSpeed > .07) {
      rotationSpeed += kF;
    } else if (rotationSpeed < -.07) {
      rotationSpeed -= kF;
    }
    driveTrainSubsystem.arcadeDrive(0.0, rotationSpeed, false);
  }

  @Override
  public boolean isFinished() {
    return noTarget || shot;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    indexerSubsystem.stopIndexer();
    driveTrainSubsystem.stop();
  }

}