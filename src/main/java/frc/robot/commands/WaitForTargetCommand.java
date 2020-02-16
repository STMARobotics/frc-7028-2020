package frc.robot.commands;

import java.time.Duration;
import java.time.LocalDateTime;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ILimelightSubsystem;

public class WaitForTargetCommand extends CommandBase {
  private final int millisecondsWait;
  private final ILimelightSubsystem[] limelights;

  private Boolean started;
  private LocalDateTime startTime;

  public WaitForTargetCommand(int millisecondsWait, ILimelightSubsystem[] limelights) {
    super();

    this.millisecondsWait = millisecondsWait;
    this.limelights = limelights;
  }

  @Override
  public void execute() {
    super.execute();

    if (!started) {
      startTime = LocalDateTime.now();
      started = true;
    }
  }

  @Override
  public boolean isFinished() {
    
    for (ILimelightSubsystem limelight : limelights) {
      if (limelight.getTargetAcquired()) {
        return true;
      }
    }

    return Duration.between(startTime, LocalDateTime.now()).toMillis() > millisecondsWait;
  }
}