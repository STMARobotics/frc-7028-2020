package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightBallCommand extends CommandBase {

  private final LimelightSubsystem limelight;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private final PIDController xPidController = new PIDController(.025, 0, 0);
  private final PIDController yPidController = new PIDController(.025, 0, 0);

  public LimelightBallCommand(DriveTrainSubsystem driveTrainSubsystem, LimelightSubsystem limelightSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.limelight = limelightSubsystem;
    addRequirements(driveTrainSubsystem, limelightSubsystem);
    
    xPidController.setSetpoint(0);
    xPidController.setTolerance(3);

    yPidController.setSetpoint(0);
    yPidController.setTolerance(1);
  }

  @Override
  public void initialize() {
    limelight.enable();
    xPidController.reset();
    yPidController.reset();
  }

  @Override
  public void execute() {
    if (limelight.getTargetAcquired()) {
      var speed = -yPidController.calculate(limelight.getTargetY());
      var rotation = -xPidController.calculate(limelight.getTargetX());
      driveTrainSubsystem.arcadeDrive(speed, rotation, false);
    } else {
      driveTrainSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return (xPidController.atSetpoint() && yPidController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveTrainSubsystem.stop();
  }

}