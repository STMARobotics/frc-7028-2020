package frc.robot.commands;

import frc.robot.subsystems.ILimelightSubsystem;

/**
 * This command won't finish until getTargetAcquired returns true, it should be used with a timeout
 */
public class WaitForTargetCommand extends VisionCommandBase {

  public WaitForTargetCommand(ILimelightSubsystem... limelights) {
    super(limelights);
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public boolean isFinished() {
    return getTargetAcquired();
  }
}