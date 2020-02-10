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
  private final LimelightSubsystem highLimelightSubsystem;
  private final LimelightSubsystem lowLimelightSubsystem;
  private boolean noTarget = false;

  private PIDController pidController = new PIDController(kP, 0, kD);

  public AimShooterCommand(LimelightSubsystem highLimelightSubsystem, LimelightSubsystem lowLimelightSubsystem,
      DriveTrainSubsystem driveTrainSubsystem) {
    this.highLimelightSubsystem = highLimelightSubsystem;
    this.lowLimelightSubsystem = lowLimelightSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(highLimelightSubsystem, lowLimelightSubsystem, driveTrainSubsystem);
    pidController.setTolerance(.01);
  }

  @Override
  public void initialize() {
    System.out.println("Targeting");
    noTarget = false;
  }

  @Override
  public void execute() {
    if (highLimelightSubsystem.getTargetAcquired()) {
      aimShooter(highLimelightSubsystem);
    } else if (lowLimelightSubsystem.getTargetAcquired()) {
      aimShooter(lowLimelightSubsystem);
    } else {
      noTarget = true;
    }
  }

  private void aimShooter(LimelightSubsystem selectedLimelightSubsystem) {
    double targetX = selectedLimelightSubsystem.getTargetX();
    double rotationSpeed = -pidController.calculate(targetX / selectedLimelightSubsystem.getMaxX());
    driveTrainSubsystem.arcadeDrive(0, rotationSpeed, false);
  }

  @Override
  public boolean isFinished() {
    return noTarget || pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Targeting complete");
    driveTrainSubsystem.stop();
  }
  
}