package frc.robot.commands;

import static frc.robot.Constants.AimConstants.kD;
import static frc.robot.Constants.AimConstants.kP;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
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
  private final DriveTrainSubsystem driveTrainSubsystem;
  
  private final PIDController pidController = new PIDController(kP, 0, kD);

  private boolean noTarget = false;
  private boolean shot = false;

  public ShootCommand(
      ShooterSubsystem shooterSubsystem, 
      IndexerSubsystem indexerSubsystem,
      LimelightSubsystem highLimelightSubsystem,
      LimelightSubsystem lowLimelightSubsystem,
      DriveTrainSubsystem driveTrainSubsystem) {
        
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.highLimelightSubsystem = highLimelightSubsystem;
    this.lowLimelightSubsystem = lowLimelightSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(shooterSubsystem);
    addRequirements(indexerSubsystem);
    addRequirements(highLimelightSubsystem);
    addRequirements(lowLimelightSubsystem);
    addRequirements(driveTrainSubsystem);

    pidController.setTolerance(.01);
  }

  @Override
  public void initialize() {
    noTarget = false;
    shot = false;
    pidController.reset();
  }

  @Override
  public void execute() {
    highLimelightSubsystem.enable();
    lowLimelightSubsystem.enable();
    if (highLimelightSubsystem.getTargetAcquired() || lowLimelightSubsystem.getTargetAcquired()) {
      if (highLimelightSubsystem.getTargetAcquired()) {
        shooterSubsystem.prepareToShoot(highLimelightSubsystem.getDistanceToTarget());
        aimShooter(highLimelightSubsystem);
      } else {
        shooterSubsystem.prepareToShoot(lowLimelightSubsystem.getDistanceToTarget());
        aimShooter(lowLimelightSubsystem);
      }
      if (shooterSubsystem.isReadyToShoot() && pidController.atSetpoint()) {
        indexerSubsystem.shoot();
        shot = true;
      }
    } else {
      noTarget = true;
    }
  }

  private void aimShooter(LimelightSubsystem selectedLimelightSubsystem) {
    double targetX = selectedLimelightSubsystem.getTargetX();
    double rotationSpeed = -pidController.calculate(targetX / selectedLimelightSubsystem.getMaxX());
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
    highLimelightSubsystem.disable();
    lowLimelightSubsystem.disable();
  }

}