package frc.robot.commands;

import static frc.robot.Constants.AimConstants.RANGE_HIGH;
import static frc.robot.Constants.AimConstants.RANGE_LOW;
import static frc.robot.Constants.AimConstants.kD;
import static frc.robot.Constants.AimConstants.kP;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * AimShooterCommand
 */
public class AimShooterCommand extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private boolean isFinished = false;

  private PIDController pidController = new PIDController(kP, 0, kD);

  public AimShooterCommand(LimelightSubsystem limelightSubsystem, DriveTrainSubsystem driveTrainSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(limelightSubsystem, driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    isFinished = false;
  }

  @Override
  public void execute() {
    if (limelightSubsystem.getTargetAcquired()) {
      double targetX = limelightSubsystem.getTargetX();
      if (targetX > RANGE_HIGH || targetX < RANGE_LOW) {
        double rotationSpeed = -pidController.calculate(targetX / limelightSubsystem.getMaxX());
        driveTrainSubsystem.arcadeDrive(0, rotationSpeed);
        return;
      }
    }
    isFinished = true;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0,0);
  }
  
}