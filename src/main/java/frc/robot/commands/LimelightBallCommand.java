package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightBallCommand extends VisionCommandBase {

  private final LimelightSubsystem limelight;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private final PIDController xPidController = new PIDController(.005, 0, 0);
  private final PIDController yPidController = new PIDController(.004, 0, 0);

  private boolean noTarget;

  public LimelightBallCommand(DriveTrainSubsystem driveTrainSubsystem, LimelightSubsystem limelightSubsystem) {
    super(limelightSubsystem);
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.limelight = limelightSubsystem;
    addRequirements(driveTrainSubsystem, limelightSubsystem);
    
    xPidController.setSetpoint(0);
    xPidController.setTolerance(3);

    yPidController.setSetpoint(0);
    yPidController.setTolerance(3);
  }

  @Override
  public void initialize() {
    xPidController.reset();
    yPidController.reset();
    noTarget = false;
  }

  @Override
  public void execute() {
    if (null == getTargetAcquired()) {
      noTarget = true;
      driveTrainSubsystem.stop();
    } else {
      var speed = yPidController.calculate(limelight.getTargetY());
      var rotation = -xPidController.calculate(limelight.getTargetX());
      driveTrainSubsystem.arcadeDrive(speed, rotation, false);
      noTarget = false;
    }
  }

  @Override
  public boolean isFinished() {
    return noTarget || (xPidController.atSetpoint() && yPidController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

}