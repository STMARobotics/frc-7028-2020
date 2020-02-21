package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ILimelightSubsystem;

public class VisionCommandBase extends CommandBase {
  private final ILimelightSubsystem[] limelights;
  private final int targetLostDelayMs;

  /**
   * Creates a new vision command with a 500 ms delay before reporting a target is lost
   * @param limelights
   */
  public VisionCommandBase(ILimelightSubsystem... limelights) {
    this(100, limelights);
  }

  public VisionCommandBase(int delayMs, ILimelightSubsystem... limelights) {
    super();

    this.limelights = limelights;
    this.targetLostDelayMs = delayMs;
  }

  @Override
  public void initialize() {
    Arrays.stream(limelights).forEach((l) -> l.enable());
  }

  @Override
  public void end(boolean interrupted) {
    Arrays.stream(limelights).forEach((l) -> l.disable());
  }

  /**
   * Returns true if tv == 1.0, once acquired delays returning false for configured amount of time
   * @return
   */
  public ILimelightSubsystem getTargetAcquired() {

    //loop the limelights and return the first successful one
    for (ILimelightSubsystem limelight : limelights) {
      //get the latest target value from limelight
      var targetValue = limelight.getRawTargetValid();

      //if we don't have a value yet then we haven't acquired the target yet
      if (targetValue == null) { 
        continue;
      }
  
      //putting a buffer on the logic here when switching from true to false, don't 'lose' the target unless we haven't seen it for N milliseconds
      if (targetValue.value == Constants.LimeLightConstants.TARGET_ACQUIRED || System.currentTimeMillis() - limelight.getTargetLastSeen() < targetLostDelayMs) {
        return limelight;
      }
    }

    //if we made it this far we don't meet the requirements
    return null;
  }
}