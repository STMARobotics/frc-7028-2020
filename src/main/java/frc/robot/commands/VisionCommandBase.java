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
    this(500, limelights);
  }

  public VisionCommandBase(int delayMs, ILimelightSubsystem... limelights) {
    super();

    this.limelights = limelights;
    this.targetLostDelayMs = delayMs;
  }

  /**
   * Returns true if tv == 1.0, once acquired delays returning false for configured amount of time
   * @return
   */
  public boolean getTargetAcquired() {
    return Arrays.stream(limelights).anyMatch(limelight -> {

      //get the latest target value from limelight
      var targetValue = limelight.getRawTargetValid();

      //if we don't have a value yet then we haven't acquired the target yet
      if (targetValue == null) { 
        return false;
      }
  
      //putting a buffer on the logic here when switching from true to false, don't 'lose' the target unless we haven't seen it for N milliseconds
      if (targetValue.Value == Constants.LimeLightConstants.TARGET_ACQUIRED || System.currentTimeMillis() - targetValue.UpdateTime < targetLostDelayMs) {
        return true;
      }
  
      //if we made it this far we don't meet the requirements
      return false;
    });
  }
}