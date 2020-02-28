package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightBallCommand extends VisionCommandBase {

  private final LimelightSubsystem limelight;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private final PIDController xPidController = new PIDController(.005, 0, 0);
  private final PIDController yPidController = new PIDController(.004, 0, 0);

  private int noTargetCount;

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
    super.initialize();
    xPidController.reset();
    yPidController.reset();
    noTargetCount = 0;
  }

  @Override
  public void execute() {
    super.execute();
    if (null == getTargetAcquired()) {
      noTargetCount++;
      driveTrainSubsystem.stop();
    } else {
      System.out.println("Y " + limelight.getTargetY());
      var speed = yPidController.calculate(limelight.getTargetY());
      var rotation = -xPidController.calculate(limelight.getTargetX());
      driveTrainSubsystem.arcadeDrive(speed, rotation, false);
      noTargetCount = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return noTargetCount > 20 || (xPidController.atSetpoint() && yPidController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveTrainSubsystem.stop();
  }

}