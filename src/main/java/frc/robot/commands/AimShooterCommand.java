package frc.robot.commands;

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
  private boolean noTarget = false;

  private PIDController pidController = new PIDController(kP, 0, kD);

  public AimShooterCommand(LimelightSubsystem limelightSubsystem, DriveTrainSubsystem driveTrainSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(limelightSubsystem, driveTrainSubsystem);
    pidController.setTolerance(.01);
  }

  @Override
  public void initialize() {
    System.out.println("Targeting");
    noTarget = false;
  }

  @Override
  public void execute() {
    if (limelightSubsystem.getTargetAcquired()) {
      double targetX = limelightSubsystem.getTargetX();
      double rotationSpeed = -pidController.calculate(targetX / limelightSubsystem.getMaxX());
      driveTrainSubsystem.arcadeDrive(0, rotationSpeed);
      return;
    }
    noTarget = true;
  }

  @Override
  public boolean isFinished() {
    return noTarget || pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Targeting complete");
    driveTrainSubsystem.arcadeDrive(0,0);
  }
  
}